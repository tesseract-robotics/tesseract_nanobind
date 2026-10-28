// Python console_bridge output handlers, delivered on their own thread.
//
// console_bridge calls the installed handler with its global output mutex held. A Python handler
// called there needs the GIL with that mutex held, and gives the GIL away again mid-call (the eval
// loop switches threads; print and sleep release it). Any thread that logs while holding the GIL
// then blocks on the mutex holding the GIL, and the handler never gets it back: a deadlock. So
// PyOutputHandler::log only copies the message into a bounded queue and returns; one delivery
// thread takes the GIL and calls the Python handlers, in the order the messages were queued.
//
// Lock order. PyOutputHandler::log runs with console_bridge's mutex held, with or without the GIL,
// and takes LogDelivery::mutex: console_bridge's mutex, then `mutex`; the GIL, then `mutex`.
// Nothing takes the GIL or console_bridge's mutex while holding `mutex`, and no logger ever waits
// for the queue: a full queue drops. (A logger blocked for space would hold console_bridge's
// mutex, which a handler that logs needs on the delivery thread.) The delivery thread takes the
// GIL holding no lock.
//
// Interpreter exit. Every call that may take the GIL (PyGILState_Ensure, and any Python code: the
// eval loop gives the GIL away and takes it back) can end the delivery thread once Python is
// finalizing. CPython 3.9-3.13 call PyThread_exit_thread there: pthread_exit, which on glibc
// unwinds the thread's C++ frames and runs their destructors (forced unwinding); 3.14 hangs the
// thread instead; macOS and Windows end it without running destructors. A destructor that touches
// Python during that unwind runs on a thread whose thread state CPython already took away: the
// GIL guard's PyGILState_Release aborted with "thread state must be current when releasing". So
// the delivery thread calls Python only through the C API, from plain functions: while it is
// inside a call that may take the GIL, no frame of it owns a Python reference, a GIL guard or a
// nanobind override ticket, and no frame is noexcept (unwinding through one terminates). A
// destructor never drops a Python reference (HandlerRef retires it); the references a thread
// ended this way still held stay allocated, as Python is going away.
#pragma once

#include "tesseract_nb.h"
#include "tesseract_common/log_flush_barrier.h"

#include <console_bridge/console.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <deque>
#include <iterator>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace tesseract_nb_common {

// Messages queued for Python handlers, at most. A message is its text (typically under 300 bytes)
// plus about 100 bytes of bookkeeping, so a full queue holds a few MB; a slow handler and fast
// loggers drop beyond it instead of growing without bound (an unbounded queue reached 5.5M
// messages, 1.7 GB).
constexpr std::size_t LOG_QUEUE_CAPACITY = 10000;

// Messages delivered per GIL acquisition. One acquisition per message cannot keep up with a
// thread logging while it holds the GIL; one per whole queue holds the GIL for up to
// LOG_QUEUE_CAPACITY handler calls and checks the exit deadline as rarely.
constexpr std::size_t LOG_DELIVERY_BATCH = 256;

// Interpreter exit: time [ms] the delivery thread gets to deliver what is queued; every message not
// yet handed to a handler then goes to stderr, and a delivery thread still inside a handler is left
// behind. Two seconds keeps exit prompt while a full queue of fast handlers (microseconds each) is
// delivered in full.
constexpr std::chrono::milliseconds LOG_EXIT_BUDGET{ 2000 };

class PyOutputHandler;

inline void retire_python_reference(PyObject* object);

// A strong reference to an installed Python handler. Created and dropped with the GIL held (by
// the swap functions and the delivery thread); a logger only copies one into the queue, never
// dropping the last copy. Its destructor does not touch Python: it retires the reference, and
// release_retired_references drops it (see "Interpreter exit" above).
struct HandlerRef {
    PyOutputHandler* handler;
    PyObject* object;           // strong
    std::uint64_t dropped = 0;  // messages to this handler dropped since its last drop notice; `mutex`

    HandlerRef(PyOutputHandler* handler_, PyObject* object_) : handler(handler_), object(object_) {}
    HandlerRef(const HandlerRef&) = delete;
    HandlerRef& operator=(const HandlerRef&) = delete;
    ~HandlerRef() { retire_python_reference(object); }
};

struct LogMessage {
    std::string text;
    console_bridge::LogLevel level;
    std::optional<std::string> filename;  // none: C++ logged a null filename, Python gets None
    int line;
    std::shared_ptr<HandlerRef> handler;  // the handler installed when it was logged
};

struct LogDelivery {
    std::mutex mutex;
    std::condition_variable pending;    // the delivery thread: a message, or stop
    std::condition_variable delivered;  // flush and stop: a batch was delivered, or stopped
    std::deque<LogMessage> queue;
    // The batch the delivery thread took off the queue; it hands in_flight[in_flight_next] to its
    // handler next. Only the delivery thread changes either, holding `mutex`; exit prints the rest.
    std::deque<LogMessage> in_flight;
    std::size_t in_flight_next = 0;
    std::uint64_t enqueued = 0;  // messages and drop notices queued, ever
    std::uint64_t delivered_count = 0;
    // Handlers with drops not yet reported (HandlerRef::dropped), each once. A logger only adds a
    // copy; a notice takes the reference over, so no logger drops one.
    std::vector<std::shared_ptr<HandlerRef>> dropped_by;
    bool delivering = false;  // a batch is off the queue, being delivered
    bool started = false;
    std::atomic<std::thread::id> thread_id{};  // read unlocked by flush
    bool stopping = false;
    std::chrono::steady_clock::time_point stop_deadline;
    // Interpreter exit: LOG_EXIT_BUDGET from the first exit hook that waits for the handlers
    // (exit_deadline_locked). One budget for all of them, so a stuck handler delays exit once.
    std::optional<std::chrono::steady_clock::time_point> exit_deadline;
    bool stopped = false;  // later messages go to stderr: Python is going away
    // A child forked in a handler whose forking thread returned from it: no atexit stops the
    // delivery thread there, which would keep the child alive. It exits whenever the queue is
    // empty instead; the next message starts another.
    bool exit_when_idle = false;
    // Mirrors console_bridge's current and previous handler, so each Python handler console_bridge
    // can call stays alive until useOutputHandler / restorePreviousOutputHandler drops it.
    std::shared_ptr<HandlerRef> current;
    std::shared_ptr<HandlerRef> previous;
    // References HandlerRef destructors let go of, for release_retired_references to drop with the
    // GIL held. A leaf lock: nothing else is taken while holding it.
    std::mutex retired_mutex;
    std::vector<PyObject*> retired;
};

// The live delivery state. Never freed: the delivery thread and late loggers may outlive module
// teardown. Replaced only in a forked child (reset_log_delivery_after_fork).
inline LogDelivery*& log_delivery_state() {
    static LogDelivery* delivery = new LogDelivery();
    return delivery;
}

inline void print_to_stderr(const std::string& text) { std::fprintf(stderr, "%s\n", text.c_str()); }

inline void retire_python_reference(PyObject* object) {
    LogDelivery& d = *log_delivery_state();
    std::lock_guard<std::mutex> lock(d.retired_mutex);
    d.retired.push_back(object);
}

// Drop the retired references, with the GIL held, from a plain function: a handler's __del__ may
// run here, so this thread may end inside a Py_DECREF at interpreter exit. The references not
// yet dropped then stay allocated.
inline void release_retired_references() {
    LogDelivery& d = *log_delivery_state();
    for (;;) {
        std::vector<PyObject*> objects;
        {
            std::lock_guard<std::mutex> lock(d.retired_mutex);
            if (d.retired.empty())
                return;
            objects.swap(d.retired);
        }
        for (PyObject* object : objects)
            Py_DECREF(object);  // a __del__ may retire more: loop
    }
}

// What the delivery thread needs to call a Python handler through the C API, set once at module
// import. base_log and levels are borrowed: the OutputHandler and LogLevel types hold them until
// interpreter finalization, and from then on no thread other than the finalizing one runs Python.
// (Strong references would keep both types alive past nanobind's leak check.)
struct PythonHandlerApi {
    PyObject* log_name = nullptr;  // "log", strong
    PyObject* base_log = nullptr;  // OutputHandler.log, the binding: a subclass that does not override it
    PyObject* levels[console_bridge::CONSOLE_BRIDGE_LOG_NONE + 1] = {};  // the LogLevel members
};

inline PythonHandlerApi& python_handler_api() {
    static PythonHandlerApi api;
    return api;
}

// At module import, with the GIL held, after OutputHandler and LogLevel are bound.
inline void init_python_handler_api(nb::handle output_handler_type) {
    PythonHandlerApi& api = python_handler_api();
    api.log_name = nb::str("log").release().ptr();
    api.base_log = nb::getattr(output_handler_type, "log").ptr();
    for (int level = console_bridge::CONSOLE_BRIDGE_LOG_DEBUG; level <= console_bridge::CONSOLE_BRIDGE_LOG_NONE;
         ++level)
        api.levels[level] = nb::cast(static_cast<console_bridge::LogLevel>(level)).ptr();
}

// The drop notice for `handler`, which takes the reference over; resets its count. With `mutex`.
inline LogMessage drop_notice(std::shared_ptr<HandlerRef> handler) {
    LogMessage notice{ "tesseract_common: dropped " + std::to_string(handler->dropped) +
                           " log messages: the Python OutputHandler fell behind (queue of " +
                           std::to_string(LOG_QUEUE_CAPACITY) + ")",
                       console_bridge::CONSOLE_BRIDGE_LOG_WARN, std::string(__FILE__), __LINE__, std::move(handler) };
    notice.handler->dropped = 0;
    return notice;
}

// Queue a drop notice for every handler with unreported drops, ahead of what is queued next, and
// count them as queued (a flush started now waits for them). With `mutex`.
template <class Sink>
inline void queue_drop_notices_locked(LogDelivery& d, Sink& sink) {
    for (auto& handler : d.dropped_by) {
        sink.push_back(drop_notice(std::move(handler)));
        ++d.enqueued;
    }
    d.dropped_by.clear();  // moved from: no reference is dropped here
}

inline void deliver_log_messages(LogDelivery* delivery);

// Start the delivery thread, with `d.mutex` held. Lazily, on the first message: also after a fork.
inline void start_log_delivery_locked(LogDelivery& d) {
    std::thread thread(deliver_log_messages, &d);
    d.thread_id.store(thread.get_id());
    thread.detach();  // never joined: exit waits a bounded time (stop_log_delivery)
    d.started = true;
}

class PyOutputHandler : public console_bridge::OutputHandler {
public:
    NB_TRAMPOLINE(console_bridge::OutputHandler, 1);

    // Called by console_bridge with its output mutex held, with or without the GIL: queue only,
    // never wait.
    void log(const std::string& text, console_bridge::LogLevel level, const char* filename, int line) override {
        LogDelivery& d = *log_delivery_state();
        std::unique_lock<std::mutex> lock(d.mutex);
        // Copy the reference only to queue it: this thread may not hold the GIL to drop one.
        const std::shared_ptr<HandlerRef>* ref = nullptr;
        if (d.current && d.current->handler == this)
            ref = &d.current;
        else if (d.previous && d.previous->handler == this)
            ref = &d.previous;
        if (d.stopped || ref == nullptr) {
            // After exit, or a handler console_bridge got around useOutputHandler: no Python call.
            lock.unlock();
            print_to_stderr(text);
            return;
        }
        if (d.queue.size() + d.dropped_by.size() + 1 > LOG_QUEUE_CAPACITY) {
            if (std::find(d.dropped_by.begin(), d.dropped_by.end(), *ref) == d.dropped_by.end())
                d.dropped_by.push_back(*ref);
            ++(*ref)->dropped;
            return;
        }
        if (!d.started)
            start_log_delivery_locked(d);
        queue_drop_notices_locked(d, d.queue);
        d.queue.push_back({ text, level,
                            filename == nullptr ? std::nullopt : std::optional<std::string>(filename), line,
                            *ref });
        ++d.enqueued;
        d.pending.notify_one();
    }

};

// The flush barrier (log_flush_barrier.h): marks on this thread, and what holds the outermost.
// Plain data, no destructor: a handler may enter one on the delivery thread (a server reload
// called from the handler), and CPython may end that thread inside it (see "Interpreter exit").
struct LogFlushBarrier {
    int depth = 0;
    const char* holder = nullptr;
};

inline LogFlushBarrier& log_flush_barrier() {
    static thread_local LogFlushBarrier barrier;
    return barrier;
}

inline void enter_log_flush_barrier(const char* holder) {
    LogFlushBarrier& barrier = log_flush_barrier();
    if (barrier.depth++ == 0)
        barrier.holder = holder;
}

inline void leave_log_flush_barrier() {
    LogFlushBarrier& barrier = log_flush_barrier();
    if (--barrier.depth == 0)
        barrier.holder = nullptr;
}

// The table behind the _log_flush_barrier capsule.
inline const LogFlushBarrierApi& log_flush_barrier_api() {
    static const LogFlushBarrierApi api{ &enter_log_flush_barrier, &leave_log_flush_barrier };
    return api;
}

// flushOutputHandlers on a thread inside a flush barrier: a handler may be waiting for it.
struct OutputHandlerFlushForbiddenError : std::runtime_error {
    using std::runtime_error::runtime_error;
};

// handler.log(text, level, filename, line) through the C API, with the GIL held (delivery thread).
// A plain function owning only raw references: CPython may end the thread inside the call (see
// "Interpreter exit" above), and the references are then left behind. A handler that raises, or a
// subclass that does not override log, is reported through sys.unraisablehook: there is no caller
// to raise into. (Called synchronously, the exception propagated into whichever C++ code logged.)
inline void call_python_handler(const LogMessage& m) {
    const PythonHandlerApi& api = python_handler_api();
    PyObject* self = m.handler->object;
    PyObject* type_log = PyObject_GetAttr(reinterpret_cast<PyObject*>(Py_TYPE(self)), api.log_name);
    if (type_log == nullptr) {
        PyErr_WriteUnraisable(self);
        return;
    }
    const bool overridden = type_log != api.base_log;
    Py_DECREF(type_log);
    if (!overridden) {
        PyErr_Format(PyExc_RuntimeError, "%R does not override OutputHandler.log",
                     reinterpret_cast<PyObject*>(Py_TYPE(self)));
        PyErr_WriteUnraisable(self);
        return;
    }
    const int level = static_cast<int>(m.level);
    if (level < console_bridge::CONSOLE_BRIDGE_LOG_DEBUG || level > console_bridge::CONSOLE_BRIDGE_LOG_NONE) {
        PyErr_Format(PyExc_ValueError, "console_bridge logged an unknown LogLevel %d", level);
        PyErr_WriteUnraisable(self);
        return;
    }
    PyObject* text = PyUnicode_FromStringAndSize(m.text.data(), static_cast<Py_ssize_t>(m.text.size()));
    PyObject* filename = nullptr;
    if (m.filename)
        filename = PyUnicode_FromStringAndSize(m.filename->data(), static_cast<Py_ssize_t>(m.filename->size()));
    else {
        filename = Py_None;
        Py_INCREF(filename);
    }
    PyObject* line = PyLong_FromLong(m.line);
    PyObject* result = nullptr;
    if (text != nullptr && filename != nullptr && line != nullptr)
        result = PyObject_CallMethodObjArgs(self, api.log_name, text, api.levels[level], filename, line, nullptr);
    Py_XDECREF(text);
    Py_XDECREF(filename);
    Py_XDECREF(line);
    if (result == nullptr)
        PyErr_WriteUnraisable(self);
    else
        Py_DECREF(result);
}

// One batch, from a plain function holding no lock, nor any object owning Python state (see
// "Interpreter exit" above). False: a forked child's copy of the delivery thread, which ends.
inline bool deliver_batch(LogDelivery& d, std::size_t taken) {
    // May end this thread at interpreter exit (3.9-3.13) or hang it (3.14).
    const PyGILState_STATE gil = PyGILState_Ensure();
    for (;;) {
        // The GIL orders this read after reset_log_delivery_after_fork in a forked child.
        if (&d != log_delivery_state()) {
            // The parent's state: its messages stay with the parent.
            {
                LogDelivery& child = *log_delivery_state();
                std::lock_guard<std::mutex> idle(child.mutex);
                child.exit_when_idle = true;
                child.pending.notify_one();
            }
            // This copy is the child's main thread (CPython made the forking thread main), and its
            // thread state the interpreter's main thread state, which must outlive the others: on
            // CPython 3.12, the next thread state made once an interpreter has none left reuses
            // the main one without resetting it, and aborts ("init_threadstate: thread state
            // already initialized"). So it keeps its thread state, as a main thread does until
            // finalization, and only gives up the GIL. The child never finalizes: it ends when its
            // last thread does.
            PyEval_SaveThread();
            return false;
        }
        const LogMessage* message = nullptr;
        {
            std::lock_guard<std::mutex> claim(d.mutex);
            if (d.stopped || d.in_flight_next == d.in_flight.size())
                break;  // stopped: exit printed the rest
            message = &d.in_flight[d.in_flight_next++];
        }
        call_python_handler(*message);
    }
    {
        std::deque<LogMessage> done;  // its HandlerRefs retire their references
        std::lock_guard<std::mutex> finish(d.mutex);
        done.swap(d.in_flight);
        d.in_flight_next = 0;
        d.delivering = false;
        d.delivered_count += taken;
        d.delivered.notify_all();
    }
    release_retired_references();
    PyGILState_Release(gil);
    return true;
}

// The delivery thread. It takes up to LOG_DELIVERY_BATCH queued messages and delivers them under
// one GIL acquisition (deliver_batch). A handler that raises is reported and the next message is
// delivered. Drops not yet reported when the queue runs empty are reported to the handlers whose
// messages were dropped, installed or not.
//
// Each message is claimed (in_flight_next) under `mutex` before it is handed over, so exit prints
// exactly the messages no handler got. A handler that forks leaves a copy of this thread as the
// child's only thread: that copy stops at once, before touching `d`, whose mutex another parent
// thread may have held; the child starts its own delivery thread on fresh state. The copy then
// ends, keeping its thread state (the child's main one; see deliver_batch), and with it the child
// once its other threads end: the child's delivery thread exits when idle from then on
// (exit_when_idle) instead of waiting forever.
inline void deliver_log_messages(LogDelivery* delivery) {
    LogDelivery& d = *delivery;
    std::unique_lock<std::mutex> lock(d.mutex);
    for (;;) {
        d.pending.wait(lock, [&] {
            return !d.queue.empty() || !d.dropped_by.empty() || d.stopping || d.exit_when_idle;
        });
        if (d.stopping && (d.queue.empty() || std::chrono::steady_clock::now() >= d.stop_deadline)) {
            // stop_log_delivery takes what is left.
            return;
        }
        if (d.exit_when_idle && d.queue.empty() && d.dropped_by.empty()) {
            d.started = false;  // the next message starts another delivery thread
            return;
        }
        const std::size_t count = std::min(d.queue.size(), LOG_DELIVERY_BATCH);
        d.in_flight.assign(std::make_move_iterator(d.queue.begin()),
                           std::make_move_iterator(d.queue.begin() + static_cast<std::ptrdiff_t>(count)));
        d.queue.erase(d.queue.begin(), d.queue.begin() + static_cast<std::ptrdiff_t>(count));
        d.in_flight_next = 0;
        d.delivering = true;
        if (d.queue.empty())
            queue_drop_notices_locked(d, d.in_flight);
        const std::size_t taken = d.in_flight.size();
        lock.unlock();
        if (!deliver_batch(d, taken))
            return;
        lock.lock();
    }
}

// flushOutputHandlers: wait, with the GIL released, until every message queued so far has been
// delivered. Inside a flush barrier it raises instead: a handler may be waiting for this thread,
// so this wait could never end. From the delivery thread (a handler) it returns at once: it would
// wait for itself. A handler that waits for the calling thread outside a barrier blocks this
// call; do not flush from code a handler may be waiting on.
inline void flush_python_output_handlers() {
    const LogFlushBarrier& barrier = log_flush_barrier();
    if (barrier.depth != 0)
        throw OutputHandlerFlushForbiddenError(
            std::string("flushOutputHandlers: called inside ") + barrier.holder +
            ", which a Python OutputHandler may be waiting for; flush before or after it");
    LogDelivery& d = *log_delivery_state();
    if (std::this_thread::get_id() == d.thread_id.load())
        return;
    nb::gil_scoped_release release;
    std::unique_lock<std::mutex> lock(d.mutex);
    // Each handler in dropped_by gets one drop notice, queued after every message counted so far.
    const std::uint64_t target = d.enqueued + d.dropped_by.size();
    d.delivered.wait(lock, [&] { return d.delivered_count >= target || d.stopped; });
}

// The exit deadline, set by the first caller. With `mutex`.
inline std::chrono::steady_clock::time_point exit_deadline_locked(LogDelivery& d) {
    if (!d.exit_deadline)
        d.exit_deadline = std::chrono::steady_clock::now() + LOG_EXIT_BUDGET;
    return *d.exit_deadline;
}

// For an exit hook that runs before stop_log_delivery (the task composer's run drain), with the
// GIL held: flushOutputHandlers within the exit budget. True when every message queued so far was
// delivered; false when the budget ran out (a slow or stuck handler) or delivery has stopped, and
// stop_log_delivery then prints the rest.
inline bool flush_python_output_handlers_at_exit() {
    LogDelivery& d = *log_delivery_state();
    if (std::this_thread::get_id() == d.thread_id.load())
        return true;
    nb::gil_scoped_release release;
    std::unique_lock<std::mutex> lock(d.mutex);
    const std::uint64_t target = d.enqueued + d.dropped_by.size();
    const auto deadline = exit_deadline_locked(d);
    return d.delivered.wait_until(lock, deadline, [&] { return d.delivered_count >= target || d.stopped; }) &&
           !d.stopped;
}

inline std::shared_ptr<HandlerRef> handler_ref(console_bridge::OutputHandler* handler) {
    auto* py = dynamic_cast<PyOutputHandler*>(handler);
    if (py == nullptr)
        return nullptr;
    PyObject* object = py->nb_trampoline.base().ptr();
    Py_INCREF(object);
    return std::make_shared<HandlerRef>(py, object);
}

// useOutputHandler. It does not wait for the queue: each queued message carries the handler it
// was logged to. The references go in before console_bridge switches, so a message logged
// meanwhile finds its handler; a reference dropped goes after the lock, with the GIL held.
inline void use_python_output_handler(console_bridge::OutputHandler* handler) {
    auto ref = handler_ref(handler);
    std::shared_ptr<HandlerRef> dropped;
    {
        LogDelivery& d = *log_delivery_state();
        std::lock_guard<std::mutex> lock(d.mutex);
        dropped = std::move(d.previous);
        d.previous = std::move(d.current);
        d.current = std::move(ref);
    }
    console_bridge::useOutputHandler(handler);
    dropped.reset();
    release_retired_references();
}

inline void restore_previous_python_output_handler() {
    {
        LogDelivery& d = *log_delivery_state();
        std::lock_guard<std::mutex> lock(d.mutex);
        std::swap(d.current, d.previous);
    }
    console_bridge::restorePreviousOutputHandler();
}

// atexit, with the GIL held: let the delivery thread deliver what is queued until the exit
// deadline (LOG_EXIT_BUDGET from the first exit hook that waited for it), then stop. Every message no handler got goes to stderr: the rest of the batch
// the delivery thread holds, the queue, a line per handler with unreported drops, and every later
// message. A delivery thread still inside a handler is left behind (it is detached); the message
// that handler has is not repeated, and the references the thread holds stay allocated, as Python
// is going away.
inline void stop_log_delivery() {
    LogDelivery& d = *log_delivery_state();
    std::deque<LogMessage> left;  // references: dropped with the GIL held, at return
    std::vector<std::string> texts;
    {
        nb::gil_scoped_release release;
        std::unique_lock<std::mutex> lock(d.mutex);
        if (d.stopped)
            return;
        d.stopping = true;
        d.stop_deadline = exit_deadline_locked(d);
        d.pending.notify_one();
        if (d.started)
            d.delivered.wait_until(lock, d.stop_deadline, [&] { return d.queue.empty() && !d.delivering; });
        d.stopped = true;  // the delivery thread hands over nothing more
        if (d.delivering) {
            for (std::size_t i = d.in_flight_next; i < d.in_flight.size(); ++i)
                texts.push_back(d.in_flight[i].text);
        }
        queue_drop_notices_locked(d, d.queue);  // after the messages logged before the drops
        left.swap(d.queue);
        d.delivered.notify_all();
    }
    for (const auto& text : texts)
        print_to_stderr(text);
    for (const auto& message : left)
        print_to_stderr(message.text);
    left.clear();
    release_retired_references();
}

// os.register_at_fork(after_in_child=...): the child has no delivery thread, and the parent's
// mutex may have been held by a thread that does not exist here. Start over with fresh state that
// keeps the installed handlers; the parent's queue stays with the parent. The child runs this with
// the GIL held and one thread, and the handler references only change with the GIL held, so
// reading them without the old mutex is safe.
inline void reset_log_delivery_after_fork() {
    LogDelivery* old = log_delivery_state();
    auto* fresh = new LogDelivery();
    fresh->current = old->current;
    fresh->previous = old->previous;
    for (auto* ref : { fresh->current.get(), fresh->previous.get() })
        if (ref != nullptr)
            ref->dropped = 0;  // the parent's drops, reported in the parent
    log_delivery_state() = fresh;  // `old` is leaked: its mutex may be held forever
}

}  // namespace tesseract_nb_common

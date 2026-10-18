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
#pragma once

#include "tesseract_nb.h"

#include <console_bridge/console.h>

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <deque>
#include <exception>
#include <iterator>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

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

// Interpreter exit: time [ms] the delivery thread gets to deliver what is queued; the rest goes to
// stderr and a delivery thread still inside a handler is left behind. Two seconds keeps exit
// prompt while a full queue of fast handlers (microseconds each) is delivered in full.
constexpr std::chrono::milliseconds LOG_EXIT_BUDGET{ 2000 };

class PyOutputHandler;

// A strong reference to an installed Python handler. Created and dropped with the GIL held (by
// the swap functions and the delivery thread); a logger only copies one into the queue, never
// dropping the last copy.
struct HandlerRef {
    PyOutputHandler* handler;
    nb::object object;
};

struct LogMessage {
    std::string text;
    console_bridge::LogLevel level;
    std::string filename;
    int line;
    std::shared_ptr<HandlerRef> handler;  // the handler installed when it was logged
};

struct LogDelivery {
    std::mutex mutex;
    std::condition_variable pending;    // the delivery thread: a message, or stop
    std::condition_variable delivered;  // flush and stop: a batch was delivered, or stopped
    std::deque<LogMessage> queue;
    std::uint64_t enqueued = 0;
    std::uint64_t delivered_count = 0;
    std::uint64_t dropped_unreported = 0;  // dropped since the last drop notice
    bool delivering = false;               // a batch is off the queue, being delivered
    bool started = false;
    std::thread::id thread_id;
    bool stopping = false;
    std::chrono::steady_clock::time_point stop_deadline;
    bool stopped = false;  // later messages go to stderr: Python is going away
    // Mirrors console_bridge's current and previous handler, so each Python handler console_bridge
    // can call stays alive until useOutputHandler / restorePreviousOutputHandler drops it.
    std::shared_ptr<HandlerRef> current;
    std::shared_ptr<HandlerRef> previous;
};

// The live delivery state. Never freed: the delivery thread and late loggers may outlive module
// teardown. Replaced only in a forked child (reset_log_delivery_after_fork).
inline LogDelivery*& log_delivery_state() {
    static LogDelivery* delivery = new LogDelivery();
    return delivery;
}

inline void print_to_stderr(const std::string& text) { std::fprintf(stderr, "%s\n", text.c_str()); }

inline LogMessage drop_notice(LogDelivery& d, std::shared_ptr<HandlerRef> handler) {
    LogMessage notice{ "tesseract_common: dropped " + std::to_string(d.dropped_unreported) +
                           " log messages: the Python OutputHandler fell behind (queue of " +
                           std::to_string(LOG_QUEUE_CAPACITY) + ")",
                       console_bridge::CONSOLE_BRIDGE_LOG_WARN, __FILE__, __LINE__, std::move(handler) };
    d.dropped_unreported = 0;
    return notice;
}

inline void deliver_log_messages(LogDelivery* delivery);

// Start the delivery thread, with `d.mutex` held. Lazily, on the first message: also after a fork.
inline void start_log_delivery_locked(LogDelivery& d) {
    std::thread thread(deliver_log_messages, &d);
    d.thread_id = thread.get_id();
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
        const std::size_t needed = d.dropped_unreported != 0 ? 2 : 1;
        if (d.queue.size() + needed > LOG_QUEUE_CAPACITY) {
            ++d.dropped_unreported;
            return;
        }
        if (!d.started)
            start_log_delivery_locked(d);
        if (d.dropped_unreported != 0) {
            d.queue.push_back(drop_notice(d, *ref));
            ++d.enqueued;
        }
        d.queue.push_back({ text, level, filename == nullptr ? std::string() : std::string(filename), line, *ref });
        ++d.enqueued;
        d.pending.notify_one();
    }

    // The Python override, with the GIL held (delivery thread).
    void deliver(const LogMessage& m) {
        NB_OVERRIDE_PURE_NAME("log", log, m.text, m.level, m.filename.c_str(), m.line);
    }
};

// The delivery thread. It takes up to LOG_DELIVERY_BATCH queued messages and delivers them under
// one GIL acquisition. A handler that raises is reported through sys.unraisablehook and the next
// message is delivered: there is no caller to raise into. (Called synchronously, the exception
// propagated into whichever C++ code logged.) Drops not yet reported when the queue runs empty
// are reported to the current handler.
inline void deliver_log_messages(LogDelivery* delivery) {
    LogDelivery& d = *delivery;
    std::unique_lock<std::mutex> lock(d.mutex);
    for (;;) {
        d.pending.wait(lock, [&] { return !d.queue.empty() || d.dropped_unreported != 0 || d.stopping; });
        if (d.stopping && (d.queue.empty() || std::chrono::steady_clock::now() >= d.stop_deadline)) {
            // stop_log_delivery takes what is left.
            return;
        }
        const std::size_t count = std::min(d.queue.size(), LOG_DELIVERY_BATCH);
        std::deque<LogMessage> batch(std::make_move_iterator(d.queue.begin()),
                                     std::make_move_iterator(d.queue.begin() + static_cast<std::ptrdiff_t>(count)));
        d.queue.erase(d.queue.begin(), d.queue.begin() + static_cast<std::ptrdiff_t>(count));
        d.delivering = true;
        std::size_t notices = 0;
        if (d.queue.empty() && d.dropped_unreported != 0 && d.current) {
            batch.push_back(drop_notice(d, d.current));
            notices = 1;
        } else if (d.queue.empty()) {
            d.dropped_unreported = 0;  // no Python handler to report to
        }
        lock.unlock();
        {
            nb::gil_scoped_acquire gil;
            for (const auto& message : batch) {
                try {
                    message.handler->handler->deliver(message);
                } catch (nb::python_error& e) {
                    e.discard_as_unraisable(message.handler->object);
                } catch (const std::exception& e) {  // e.g. a subclass without a log override
                    PyErr_SetString(PyExc_RuntimeError, e.what());
                    PyErr_WriteUnraisable(message.handler->object.ptr());
                }
            }
            batch.clear();  // handler references go with the GIL held
        }
        lock.lock();
        d.delivering = false;
        d.delivered_count += count;
        d.enqueued += notices;
        d.delivered_count += notices;
        d.delivered.notify_all();
    }
}

// flushOutputHandlers: wait, with the GIL released, until every message queued so far has been
// delivered. From the delivery thread (a handler) it returns at once: it would wait for itself.
// A handler that waits for the calling thread blocks this call; do not flush from code a handler
// may be waiting on.
inline void flush_python_output_handlers() {
    LogDelivery& d = *log_delivery_state();
    if (std::this_thread::get_id() == d.thread_id)
        return;
    nb::gil_scoped_release release;
    std::unique_lock<std::mutex> lock(d.mutex);
    const std::uint64_t target = d.enqueued;
    d.delivered.wait(lock, [&] { return d.delivered_count >= target || d.stopped; });
}

inline std::shared_ptr<HandlerRef> handler_ref(console_bridge::OutputHandler* handler) {
    auto* py = dynamic_cast<PyOutputHandler*>(handler);
    if (py == nullptr)
        return nullptr;
    return std::make_shared<HandlerRef>(HandlerRef{ py, nb::borrow(py->nb_trampoline.base()) });
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
}

inline void restore_previous_python_output_handler() {
    {
        LogDelivery& d = *log_delivery_state();
        std::lock_guard<std::mutex> lock(d.mutex);
        std::swap(d.current, d.previous);
    }
    console_bridge::restorePreviousOutputHandler();
}

// atexit, with the GIL held: let the delivery thread deliver what is queued for at most
// LOG_EXIT_BUDGET, then stop. What is left goes to stderr, and so does every later message. A
// delivery thread still inside a handler is left behind (it is detached); the messages it holds
// and the handler references stay allocated, as Python is going away.
inline void stop_log_delivery() {
    LogDelivery& d = *log_delivery_state();
    std::deque<LogMessage> left;
    {
        nb::gil_scoped_release release;
        std::unique_lock<std::mutex> lock(d.mutex);
        if (d.stopped)
            return;
        d.stopping = true;
        d.stop_deadline = std::chrono::steady_clock::now() + LOG_EXIT_BUDGET;
        d.pending.notify_one();
        if (d.started)
            d.delivered.wait_until(lock, d.stop_deadline, [&] { return d.queue.empty() && !d.delivering; });
        d.stopped = true;
        left.swap(d.queue);
        d.delivered.notify_all();
    }
    for (const auto& message : left)
        print_to_stderr(message.text);
    // Holding the GIL: dropping the references is safe.
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
    log_delivery_state() = fresh;  // `old` is leaked: its mutex may be held forever
}

}  // namespace tesseract_nb_common

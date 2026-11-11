/**
 * @file common_logging.cpp
 * @brief tesseract_common: spdlog-backed tesseract logging (upstream #1367)
 */

#include "common_bindings.h"

#include <algorithm>
#include <utility>

#include <tesseract/common/logging.h>
#include <nanobind/stl/chrono.h>
#include <nanobind/stl/string_view.h>
#include <spdlog/spdlog.h>
#include <spdlog/details/null_mutex.h>
#include <spdlog/sinks/base_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>

namespace {

// A Python callable as upstream's LogRecordHandler. Upstream calls handlers from whichever
// thread logs and swallows their exceptions, so this takes the GIL, hands Python a copy of
// the record, and reports a raising handler through sys.unraisablehook. The callable is
// released under the GIL (removeLogRecordHandler runs without it); after interpreter
// finalization there is no Python left to call or to decref, so it is leaked instead.
class PyLogRecordHandler {
public:
    explicit PyLogRecordHandler(nb::callable fn)
        : fn_(new nb::object(std::move(fn)), &PyLogRecordHandler::release) {}

    void operator()(const tesseract::common::LogRecord& record) const {
        if (!Py_IsInitialized())
            return;
        nb::gil_scoped_acquire gil;
        try {
            (*fn_)(nb::cast(record, nb::rv_policy::copy));
        } catch (nb::python_error& e) {
            e.discard_as_unraisable("tesseract log record handler");
        }
    }

private:
    static void release(nb::object* fn) {
        if (!Py_IsInitialized()) {
            fn->release();
            delete fn;
            return;
        }
        nb::gil_scoped_acquire gil;
        delete fn;
    }

    std::shared_ptr<nb::object> fn_;
};

// An spdlog sink that writes each record to Python's logging module, on the logger named at
// construction. The text is the sink's pattern applied to the record ("%v", the message, by
// default); the level, source file, line and function become the Python LogRecord's. spdlog
// calls a sink from whichever thread logs, so this takes the GIL, as PyLogRecordHandler does,
// and reports a raising handler through sys.unraisablehook. It holds no Python object: after
// interpreter finalization a record is dropped.
//
// The GIL is this sink's only lock. A mutex here would be taken before the GIL on a logging
// thread and after it on a thread already holding the GIL (Logger.flush, any binding that
// logs), which deadlocks, and a Python handler that logs would re-lock it on its own thread.
// The formatter runs under the GIL, and Python only sets the pattern while holding it.
//
// Ownership: every sink Python can hold is created by C++ (std::make_shared, see nb::new_ below)
// and can hand out its own shared_ptr (SharedSink). A logger given one from Python then shares
// that C++ ownership; it never holds a reference to a Python object, so tesseract's default
// logger, which outlives the interpreter, keeps nothing of Python's alive.
struct SharedSink : std::enable_shared_from_this<SharedSink> {
    virtual ~SharedSink() = default;
};

class PythonLoggingSink final : public spdlog::sinks::base_sink<spdlog::details::null_mutex>, public SharedSink {
public:
    explicit PythonLoggingSink(std::string name) : name_(std::move(name)) {
        set_pattern_("%v");
    }

    const std::string& name() const { return name_; }

protected:
    void sink_it_(const spdlog::details::log_msg& msg) override {
        if (!Py_IsInitialized())
            return;
        nb::gil_scoped_acquire gil;
        spdlog::memory_buf_t text;
        formatter_->format(msg, text);
        std::size_t size = text.size();
        while (size > 0 && (text[size - 1] == '\n' || text[size - 1] == '\r'))
            --size;
        try {
            const int level = pythonLevel(msg.level);
            nb::object logger = nb::module_::import_("logging").attr("getLogger")(name_);
            if (!nb::cast<bool>(logger.attr("isEnabledFor")(level)))
                return;
            nb::dict extra;
            extra["spdlog_logger"] = std::string(msg.logger_name.data(), msg.logger_name.size());
            nb::object record = logger.attr("makeRecord")(
                logger.attr("name"), level, msg.source.filename ? msg.source.filename : "", msg.source.line,
                std::string(text.data(), size), nb::tuple(), nb::none(),
                msg.source.funcname ? nb::object(nb::str(msg.source.funcname)) : nb::none(), extra);
            logger.attr("handle")(record);
        } catch (nb::python_error& e) {
            e.discard_as_unraisable("tesseract PythonLoggingSink");
        }
    }

    void flush_() override {}

private:
    // Python has no TRACE: 5 sits below DEBUG, as logging's own docs suggest for finer levels.
    static int pythonLevel(spdlog::level::level_enum level) {
        switch (level) {
            case spdlog::level::trace: return 5;
            case spdlog::level::debug: return 10;
            case spdlog::level::info: return 20;
            case spdlog::level::warn: return 30;
            case spdlog::level::err: return 40;
            case spdlog::level::critical: return 50;
            default: return 0;
        }
    }

    std::string name_;
};

// spdlog's colored stderr sink, as Python constructs it: shareable as PythonLoggingSink is.
// Bound as spdlog's own class, so tesseract's default sink is a StderrSink in Python too.
class SharedStderrSink final : public spdlog::sinks::stderr_color_sink_mt, public SharedSink {};

}  // namespace

void bind_logging(nb::module_& m) {
    // ========== Logging (spdlog, upstream #1367) ==========
    // Mirrors upstream: getLogger(name) for the logger, its level and its spdlog sinks;
    // addLogRecordHandler for a callable that sees every record as well.
    // Replaces the console_bridge API, which no longer reached tesseract's output.
    nb::enum_<spdlog::level::level_enum>(m, "LogLevel")
        .value("trace", spdlog::level::trace)
        .value("debug", spdlog::level::debug)
        .value("info", spdlog::level::info)
        .value("warn", spdlog::level::warn)
        .value("err", spdlog::level::err)
        .value("critical", spdlog::level::critical)
        .value("off", spdlog::level::off);

    // Sinks Python has read from a logger, so assigning one back shares its C++ ownership
    // (guarded by the GIL; expired entries are pruned on each read).
    static std::vector<std::weak_ptr<spdlog::sinks::sink>> read_sinks;
    // The C++ shared_ptr behind a sink Python holds; never one that keeps a Python object alive.
    const auto cpp_sink = [](nb::handle h) -> spdlog::sink_ptr {
        if (nb::isinstance<spdlog::sinks::sink>(h)) {
            auto* raw = nb::cast<spdlog::sinks::sink*>(h);
            if (auto* shared = dynamic_cast<SharedSink*>(raw))
                return spdlog::sink_ptr(shared->shared_from_this(), raw);
            for (const auto& weak : read_sinks)
                if (auto sink = weak.lock(); sink.get() == raw)
                    return sink;
        }
        throw nb::type_error("a sink must be a StderrSink, a PythonLoggingSink or one read from Logger.sinks");
    };
    // Sinks are where a logger writes. A record goes to every sink of the logger whose level
    // admits it, after the logger's own level has.
    nb::class_<spdlog::sinks::sink>(m, "LogSink", "An spdlog sink: one place a Logger writes its records")
        .def("level", &spdlog::sinks::sink::level)
        .def("set_level", &spdlog::sinks::sink::set_level, "level"_a,
             "Drop records below this level at this sink only")
        .def("set_pattern", &spdlog::sinks::sink::set_pattern, "pattern"_a,
             "Format records with an spdlog pattern (e.g. '%v (%g:%#)' for the message and its source)");
    nb::class_<spdlog::sinks::stderr_color_sink_mt, spdlog::sinks::sink>(
        m, "StderrSink", "Colored standard error, the kind of sink tesseract's default logger starts with")
        .def(nb::new_([]() -> std::shared_ptr<spdlog::sinks::stderr_color_sink_mt> {
            return std::make_shared<SharedStderrSink>();
        }));
    nb::class_<PythonLoggingSink, spdlog::sinks::sink>(
        m, "PythonLoggingSink",
        "Writes each record to Python's logging module, on the logger named `name`, with its level,\n"
        "source file, line and function; the text is the sink's pattern ('%v', the message, by default).\n"
        "Records the Python logger is not enabled for are dropped there. Takes the GIL on the logging thread.")
        .def(nb::new_([](std::string name) { return std::make_shared<PythonLoggingSink>(std::move(name)); }),
             "name"_a = "tesseract")
        .def_prop_ro("name", &PythonLoggingSink::name, "The Python logger records go to");

    nb::class_<spdlog::logger>(m, "Logger", "A named spdlog logger (tesseract's default is 'tesseract')")
        .def("name", [](const spdlog::logger& self) { return self.name(); })
        .def("level", &spdlog::logger::level)
        .def("set_level", &spdlog::logger::set_level, "level"_a)
        .def("should_log", &spdlog::logger::should_log, "level"_a)
        .def_prop_rw(
            "sinks",
            [](spdlog::logger& self) {
                read_sinks.erase(std::remove_if(read_sinks.begin(), read_sinks.end(),
                                                [](const auto& weak) { return weak.expired(); }),
                                 read_sinks.end());
                for (const auto& sink : self.sinks())
                    read_sinks.push_back(sink);
                return self.sinks();
            },
            [cpp_sink](spdlog::logger& self, nb::sequence sinks) {
                std::vector<spdlog::sink_ptr> converted;
                for (nb::handle sink : sinks)
                    converted.push_back(cpp_sink(sink));
                self.sinks() = std::move(converted);
            },
            "The logger's sinks, as a list. Assign a new list to change them, before other threads\n"
            "log (spdlog's sink list is not thread safe), e.g. at startup. A logger created later by\n"
            "getLogger(name) copies the default logger's sinks.")
        .def("set_pattern", [](spdlog::logger& self, const std::string& pattern) { self.set_pattern(pattern); },
             "pattern"_a, "Set the pattern of every sink")
        .def("flush", &spdlog::logger::flush);

    using tesseract::common::LogRecord;
    nb::class_<LogRecord>(m, "LogRecord", "One tesseract log event, as passed to a record handler")
        .def_ro("timestamp", &LogRecord::timestamp)
        .def_ro("level", &LogRecord::level)
        .def_ro("logger_name", &LogRecord::logger_name)
        .def_ro("component_name", &LogRecord::component_name)
        .def_ro("message", &LogRecord::message)
        .def_ro("attributes", &LogRecord::attributes)
        .def_prop_ro("filename", [](const LogRecord& self) {
            return std::string(self.source_location.filename ? self.source_location.filename : "");
        })
        .def_prop_ro("line", [](const LogRecord& self) { return self.source_location.line; })
        .def_prop_ro("function_name", [](const LogRecord& self) {
            return std::string(self.source_location.funcname ? self.source_location.funcname : "");
        });

    m.def("getLogger", &tesseract::common::getLogger, "name"_a = "tesseract",
          "Get (or create) a tesseract spdlog logger; set_level() on it controls tesseract's output");
    m.def("isLogLevelEnabled", &tesseract::common::isLogLevelEnabled, "level"_a,
          "Whether the default tesseract logger emits at this level");
    // Ids of the handlers registered from Python, guarded by the GIL. At interpreter exit the
    // ones still registered are removed, so their callables (and whatever they captured) are
    // released while Python is still alive rather than leaked at static destruction.
    static std::set<tesseract::common::LogRecordHandlerId> py_handler_ids;
    m.def("addLogRecordHandler",
          [](nb::callable handler) {
              const auto id = tesseract::common::addLogRecordHandler(PyLogRecordHandler(std::move(handler)));
              py_handler_ids.insert(id);
              return id;
          },
          "handler"_a,
          "Register handler(record: LogRecord), called for every emitted record; returns its id");
    m.def("removeLogRecordHandler",
          [](tesseract::common::LogRecordHandlerId id) {
              py_handler_ids.erase(id);
              // Upstream waits here for the handler's in-flight calls on other threads, which need the GIL.
              nb::gil_scoped_release release;
              return tesseract::common::removeLogRecordHandler(id);
          },
          "handler_id"_a,
          "Unregister a handler by id; False if the id is not registered");
    nb::module_::import_("atexit").attr("register")(nb::cpp_function([]() {
        const auto ids = std::exchange(py_handler_ids, {});
        nb::gil_scoped_release release;
        for (const auto id : ids)
            tesseract::common::removeLogRecordHandler(id);
    }));

}

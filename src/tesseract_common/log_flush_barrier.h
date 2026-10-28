// The flush barrier: a per-thread mark "a Python log handler may be waiting for this thread".
//
// flushOutputHandlers waits for the delivery thread, which runs the Python OutputHandlers. A
// handler may itself wait for another thread: a TaskComposerServer call from a handler waits for
// a reload running on another thread (the server gate). That reload must then never wait for the
// delivery thread, or both wait forever. tesseract_common cannot see who waits for whom, so the
// module that makes a handler wait marks the thread it waits for: between enter and leave,
// flushOutputHandlers on that thread raises OutputHandlerFlushForbiddenError, naming `holder`,
// instead of waiting. The marks nest.
//
// tesseract_common owns the mark (python_output_handler.h) and exports this table as the
// `_log_flush_barrier` capsule; a module that waits on something a handler may call (the task
// composer's server gate) imports it. The dependency points the way the modules already do.
#pragma once

namespace tesseract_nb_common {

inline constexpr const char* LOG_FLUSH_BARRIER_CAPSULE = "tesseract_robotics.log_flush_barrier";

struct LogFlushBarrierApi {
    // Mark the calling thread. `holder` names what holds it in the error (static storage: a
    // string literal). Never throws.
    void (*enter)(const char* holder);
    // Undo the matching enter on the calling thread. Never throws.
    void (*leave)();
};

}  // namespace tesseract_nb_common

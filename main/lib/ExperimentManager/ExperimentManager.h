#ifndef EXPERIMENT_MANAGER_H
#define EXPERIMENT_MANAGER_H

#include <stddef.h>
#include <stdint.h>

enum class ExperimentContent : uint8_t {
    WARMUP = 0,
    WALK = 1,
    ENDING = 2
};

enum class ErrorAction : uint8_t {
    RESTART,
    SKIP
};

enum class ExperimentState : uint8_t {
    RUNNING,
    RECOVERING,
    END
};

struct ExperimentProcedureItem {
    float t_sup_start;
    float t_sup_end;
    size_t step_count;
    ErrorAction error_action;
    ExperimentContent content;
};

class ExperimentManager {
public:
    ExperimentManager();
    ExperimentManager(
        const ExperimentProcedureItem* procedure,
        size_t procedure_count);

    void configure(
        const ExperimentProcedureItem* procedure,
        size_t procedure_count);
    void reset();

    // Call once when a single-support step has completed and double support begins.
    // Returns true when the procedure item changed or the experiment reached END.
    bool on_step_completed();

    // Apply the current procedure item's error action and enter RECOVERING.
    void on_fall();
    void on_recovery_completed();

    ExperimentState get_state() const { return state; }
    bool is_running() const { return state == ExperimentState::RUNNING; }
    bool is_end() const { return state == ExperimentState::END; }

    float get_t_sup() const;
    ExperimentContent get_content() const;
    ErrorAction get_error_action() const;
    size_t get_procedure_index() const { return procedure_index; }
    size_t get_step_in_procedure() const { return step_in_procedure; }
    size_t get_total_step_count() const { return total_step_count; }
    size_t get_attempt_id() const { return attempt_id; }

private:
    const ExperimentProcedureItem& active_or_last_item() const;

    const ExperimentProcedureItem* procedure;
    size_t procedure_count;
    size_t procedure_index = 0;
    size_t step_in_procedure = 0;
    size_t total_step_count = 0;
    size_t attempt_id = 0;
    ExperimentState state = ExperimentState::END;
};

#endif

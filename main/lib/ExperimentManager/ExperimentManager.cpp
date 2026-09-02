#include "ExperimentManager.h"

ExperimentManager::ExperimentManager()
    : procedure(nullptr), procedure_count(0)
{
    reset();
}

ExperimentManager::ExperimentManager(
    const ExperimentProcedureItem* procedure,
    size_t procedure_count)
    : procedure(procedure), procedure_count(procedure_count)
{
    reset();
}

void ExperimentManager::configure(
    const ExperimentProcedureItem* new_procedure,
    size_t new_procedure_count)
{
    procedure = new_procedure;
    procedure_count = new_procedure_count;
    reset();
}

void ExperimentManager::reset(){
    procedure_index = 0;
    step_in_procedure = 0;
    total_step_count = 0;
    attempt_id = 0;
    state = (procedure != nullptr && procedure_count > 0)
        ? ExperimentState::RUNNING
        : ExperimentState::END;
}

bool ExperimentManager::on_step_completed(){
    if (state != ExperimentState::RUNNING ||
        procedure == nullptr || procedure_index >= procedure_count) {
        return false;
    }

    total_step_count++;
    step_in_procedure++;

    if (step_in_procedure < procedure[procedure_index].step_count) {
        return false;
    }

    procedure_index++;
    step_in_procedure = 0;
    if (procedure_index >= procedure_count) {
        state = ExperimentState::END;
    }
    return true;
}

void ExperimentManager::on_fall(){
    if (state != ExperimentState::RUNNING ||
        procedure == nullptr || procedure_index >= procedure_count) {
        return;
    }

    if (procedure[procedure_index].error_action == ErrorAction::RESTART) {
        step_in_procedure = 0;
        attempt_id++;
    }else{
        procedure_index++;
        step_in_procedure = 0;
    }

    state = ExperimentState::RECOVERING;
}

void ExperimentManager::on_recovery_completed(){
    if (state != ExperimentState::RECOVERING) {
        return;
    }

    state = (procedure != nullptr && procedure_index < procedure_count)
        ? ExperimentState::RUNNING
        : ExperimentState::END;
}

const ExperimentProcedureItem& ExperimentManager::active_or_last_item() const{
    static const ExperimentProcedureItem empty_item = {
        0.0f,
        0.0f,
        0,
        ErrorAction::RESTART,
        ExperimentContent::WARMUP
    };
    if (procedure == nullptr || procedure_count == 0) {
        return empty_item;
    }

    const size_t index = procedure_index < procedure_count
        ? procedure_index
        : procedure_count - 1;
    return procedure[index];
}

float ExperimentManager::get_t_sup() const{
    const ExperimentProcedureItem& item = active_or_last_item();
    if (procedure == nullptr || procedure_count == 0) {
        return 0.0f;
    }

    if (state == ExperimentState::END &&
        procedure_index >= procedure_count) {
        return item.t_sup_end;
    }

    if (item.step_count <= 1 || item.t_sup_start == item.t_sup_end) {
        return item.t_sup_end;
    }

    const float progress =
        static_cast<float>(step_in_procedure) /
        static_cast<float>(item.step_count - 1);
    return item.t_sup_start +
        (item.t_sup_end - item.t_sup_start) * progress;
}

ExperimentContent ExperimentManager::get_content() const{
    return active_or_last_item().content;
}

ErrorAction ExperimentManager::get_error_action() const{
    return active_or_last_item().error_action;
}

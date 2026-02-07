/**
 * @file ui_manager.h
 * @brief UI Manager interface
 *
 * Main UI thread that coordinates display, input, and timer state machine.
 */

#ifndef UI_MANAGER_H_
#define UI_MANAGER_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the UI manager
 *
 * Initializes display, input, and timer subsystems.
 * Must be called before creating the UI thread.
 *
 * @return 0 on success, negative errno on failure
 */
int ui_manager_init(void);

/**
 * @brief UI thread entry point
 *
 * This function runs the main UI event loop. It should be called
 * from a dedicated thread.
 *
 * @param arg1 Unused
 * @param arg2 Unused
 * @param arg3 Unused
 */
void ui_manager_thread(void *arg1, void *arg2, void *arg3);

#ifdef __cplusplus
}
#endif

#endif /* UI_MANAGER_H_ */

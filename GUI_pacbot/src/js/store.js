import { reactive } from 'vue'

export const store = reactive({
    plan: [],
    robot_can_move: false,
    robot_motion_status: false,
    emergency: false,
    arm: null,
    pattern: null,
    robot_speed: 200,
    logs: [
        {
            time_stamp: new Date().toISOString(),
            message: " ============{ Logs }============",
            color: "white",
            name: 0,
        }
    ],
})
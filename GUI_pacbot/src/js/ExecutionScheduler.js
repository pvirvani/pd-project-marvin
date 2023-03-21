import { store } from "./store.js";
import ROSInterface from "./ROSInterface.js";

export default class PlanScheduler {
  constructor(emitter) {
    this.iface = new ROSInterface();
    this.old_status = false;
    this.emitter = emitter;
    this.action_id = 0;
    this.motionStatus();
  }

  init = () => {
    if (this.action_id < store.plan.length) {
      if (
        store.plan[this.action_id].actor === "robot_arm" &&
        store.robot_can_move &&
        store.plan[this.action_id].status === "pending"
      ) {
        store.plan[this.action_id].status = "current";

        let lego_ = store.plan[this.action_id].lego;
        let pick_pos = store.plan[this.action_id].pick;
        let place_pos = store.plan[this.action_id].place;
        let rot_ = store.plan[this.action_id].rot;
        this.emitter.emit("action-execution", { lego_, place_pos, rot_ });

        this.execute_action(pick_pos, place_pos);
        store.plan[this.action_id].status = "executed";

        const log = {
          time_stamp: new Date().toISOString(),
          message:
            "Action P&P [" +
            store.plan[this.action_id].lego +
            "] Was executed by the Robot!",
          color: "white",
          name: store.logs.length,
        };
        store.logs.push(log);

        this.action_id += 1;
      } else if (store.plan[this.action_id].actor === "operator") {
        this.action_id += 1;
        this.init();
      }
    }
  };

  execute_action = (pick_, place_) => {
    const service_name = "/pick_place_rapid";
    const service_type = "yumi_rapid_interface/PickPlaceRAPID";
    let robot_arm = store.arm === "left";
    const params_ = {
      pick: pick_,
      place: place_,
      speed: Number(store.robot_speed),
      arm: robot_arm,
    };
    this.iface.callService_(service_name, service_type, params_);
  };

  motionStatusCallback = (msg) => {
    if (this.old_status !== msg) {
      store.robot_motion_status = msg;
      this.old_status = msg;
      if (!msg) {
        this.init();
      }
    }
  };

  motionStatus = () => {
    this.iface.subscriber(
      "/yumi_motion_status",
      "std_msgs/Bool",
      this.motionStatusCallback
    );
  };
}

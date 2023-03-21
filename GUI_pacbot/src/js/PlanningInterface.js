import ROSInterface from "./ROSInterface.js";
import { store } from "./store.js";

export default class PlanningInterface {
  constructor() {
    this.iface = new ROSInterface();
    this.plan_msg = "";
    this.lego_map_msg = "";
    this.lego_map = null;
    this.plan = [];
    this.colors = {
      r: "red",
      g: "green",
      b: "blue",
      y: "yellow",
      w: "white",
      l: "light_green",
      o: "olive",
    };
    this.updatePlan();
    this.action_id = 0;
  }

  rosPlanIface = (msg) => {
    if (this.plan_msg.localeCompare(msg) != 0 && this.lego_map !== null) {
      this.plan_msg = msg;
      msg = msg.trim().replace(new RegExp("'", "g"), '"');
      const plan = JSON.parse(msg);
      /**
       * Create Actions
       */
      for (let act_id in plan) {
        let action = {};
        action.lego = plan[act_id][1];
        action.act_id = this.action_id;
        this.action_id += 1;
        action.actor = plan[act_id][0];
        let p_xx_yy_zz = plan[act_id][2].split("_");
        action.place = {};
        action.place.x = Number(p_xx_yy_zz[1]);
        action.place.y = Number(p_xx_yy_zz[2]);
        action.place.z = Number(p_xx_yy_zz[3]);

        if (plan[act_id].length === 3) {
          action.size = 2;
        } else if (plan[act_id].length === 5) {
          action.size = 4;
        }

        if (this.lego_map !== null && this.lego_map[action.lego]) {
          let p_xx_yy = this.lego_map[action.lego][0].split("_");
          action.pick = {};
          action.pick.x = Number(p_xx_yy[1]);
          action.pick.y = Number(p_xx_yy[2]);
          action.pick.z = this.lego_map[action.lego][action.size - 1] - 1;
          action.color = this.colors[action.lego[0]];
          action.status = "pending";
          /**
           * Operator Executed Actions
           */
          if (action.pick.x < 7 && action.actor === "operator") {
            action.changed = true;
            action.status = "executed";

            const log = {
              time_stamp: new Date().toISOString(),
              message:
                "Action P&P [" +
                action.lego +
                "] Was executed by the Operator!",
              color: "white",
              name: store.logs.length,
            };
            store.logs.push(log);
          }
          action.rot = false;
          if (action.size === 4) {
            action.pick.z = this.lego_map[action.lego][action.size - 2] - 1;
            action.rot = this.lego_map[action.lego][action.size - 1];
          }
          this.plan.push(action);
        } else {
          console.log(action.lego + " is not found in lego map ");
        }
      }
      /**
       * Check if the action already exists in the old plan.
       */
      for (let id_ in this.plan) {
        let lego_ = this.plan[id_].lego;
        let action_exist = -1;
        for (let id2 in store.plan) {
          if (Object.values(store.plan[id2]).includes(lego_)) {
            action_exist = id2;
          }
        }
        /**
         * This action exists in the old plan
         */
        if (action_exist !== -1) {
          /**
           * Changed Actions (executed, assigned to operator)
           * should not be removed!
           */
          if (!store.plan[action_exist].changed) {
            store.plan[action_exist] = this.plan[id_];
          }
        } else {
          /**
           * New Action
           */
          store.plan.push(this.plan[id_]);
        }
      }
    }
  };

  rosVisionIface = (msg) => {
    if (this.lego_map_msg.localeCompare(msg) !== 0) {
      this.lego_map_msg = msg;
      const msg_obj = JSON.parse(msg);
      this.lego_map = msg_obj;
    }
  };

  updatePlan = () => {
    // create a listener to retrieve world model from camera
    this.iface.subscriber("/plan", "std_msgs/String", this.rosPlanIface);
    this.iface.subscriber("/lego_map", "std_msgs/String", this.rosVisionIface);
  };

  getPlan = () => {
    return this.plan;
  };
}

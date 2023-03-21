<!-- 
    Action.vue 
    ---
    @author: Lukas Loiodice.
    LIG/Marvin France, 2022. 
    ---
    @modified by: Belal HMEDAN.
    LIG/Marvin France, 2022.
-->
<template>
  <!-- Plan List Draggable -->
  <li>
    <VueInteractDraggable
      :interact-max-rotation="20"
      @draggedLeft="drag('left')"
      @draggedRight="drag('right')"
      :interact-x-threshold="100"
      :interact-lock-x-axis="blocked"
      :interact-lock-y-axis="true"
      v-if="isShowing"
    >
      <div @dblclick="choose()" :class="action_style">
        <div class="block">
          <h2>{{ action_id }}</h2>
        </div>
        <div class="block">
          <img
            :src="require(`../assets/legos/2x${size}${color}.png`)"
            alt="legos"
            id="lego"
          />
        </div>
        <div class="block" :id="action_id">
          <h3>{{ actor }}</h3>
        </div>
      </div>
    </VueInteractDraggable>
  </li>
</template>

<script>
import VueInteractDraggable from "./VueInteractDraggable.vue";
import { store } from "../js/store.js";

export default {
  name: "ActionComp",
  emits: ["fix", "error"],
  components: { VueInteractDraggable },
  data() {
    return {
      store,
      isShowing: true,
    };
  },
  props: {
    /* Action ID */
    action_id: {
      type: Number,
      required: true,
    },
    /* Actor (Human Robot Left/Right Arm) */
    actor: {
      type: String,
      required: true,
    },
    /* Lego Name */
    lego_name: {
      type: String,
      required: true,
    },
    pick_pos: {
      type: Object,
      required: true,
    },
    place_pos: {
      type: Object,
      required: true,
    },
    /* Lego Color */
    color: {
      type: String,
      required: true,
    },
    /* Lego Size 2, 4, 6 */
    size: {
      type: Number,
      required: true,
    },
    status: {
      type: String,
      required: true,
    },
  },
  computed: {
    action_style() {
      for (let id_ in store.plan) {
        if (Number(id_) === this.action_id) {
          /**********
           * executed
           **********/
          if (store.plan[id_].status === "executed") {
            return "deactivated";
          } else if (store.plan[id_].status === "current") {
            /**********
             * current
             **********/
            return "current";
          } else {
            /**********
             * pending
             **********/
            if (store.plan[id_].actor === "operator") {
              return "operator";
            } else {
              return "robot";
            }
          }
        }
      }
    },
    blocked() {
      return (this.status === "pending" && this.actor === "robot_arm") ? false : true;
    },
  },
  methods: {
    /* Choose Action from Plan */
    choose() {
      this.emitter.emit("choose-action", {
        place_pos: this.place_pos,
        pick_pos: this.pick_pos,
        action_id: this.action_id,
      });
      const log = {
        time_stamp: new Date().toISOString(),
        message:
          this.action_id +
          ": " +
          this.lego_name +
          "_actor: " +
          this.actor +
          " was clicked.",
        color: "white",
        name: store.logs.length,
      };
      store.logs.push(log);
    },

    drag(side) {
      this.$emit("fix", {
        act_id: this.action_id,
        actor: this.actor,
      });
      const log = {
        time_stamp: new Date().toISOString(),
        message:
          this.action_id +
          ": " +
          this.lego_name +
          " Pick (" +
          String(this.pick_pos.x) +
          "_" +
          String(this.pick_pos.y) +
          "_" +
          String(this.pick_pos.z) +
          ") & Place (" +
          String(this.pick_pos.x) +
          "_" +
          String(this.pick_pos.y) +
          "_" +
          String(this.pick_pos.z) +
          ") by: " +
          this.actor +
          " ( drag: " +
          side +
          " )",
        color: "orange",
        name: store.logs.length,
      };
      store.logs.push(log);

      setTimeout(() => {
        this.isShowing = false;
      }, 100);
      setTimeout(() => {
        this.isShowing = true;
      }, 400);
    },
  },
};
</script>

<style>
@import "../style/css/ActionComp.css";
</style>

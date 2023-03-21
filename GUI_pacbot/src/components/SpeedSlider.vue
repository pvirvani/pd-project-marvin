<!-- 
    SpeedSlider.vue 
    ---
    @author: Lukas Loiodice.
    LIG/Marvin France, 2022. 
    ---
    @modified by: Belal HMEDAN.
    LIG/Marvin France, 2022.
    ---
-->
<template>
  <div class="range tooltip" style="--step: 50; --min: 50; --max: 400">
    <label>
      <p>Speed of the robot</p>
    </label>
    <input
      @input="updateSpeed()"
      v-model="store.robot_speed"
      type="range"
      min="50"
      max="400"
      step="50"
    />
    <div class="tooltiptext">
      Slide me to increase or decrease the speed of the robot
    </div>
  </div>
</template>

<script>
import { store } from "../js/store.js";

export default {
  name: "SpeedSlider",
  data() {
    return {
      store,
    };
  },
  methods: {
    updateSpeed() {
      const log = {
        time_stamp: new Date().toISOString(),
        message:
          "Robot Speed Has been changed to: ( " +
          store.robot_speed +
          " mm/sec)",
        color: "orange",
        name: store.logs.length,
      };
      store.logs.push(log);
    },
  },
};
</script>

<style>
@import "../style/css/SpeedSlider.css";
</style>

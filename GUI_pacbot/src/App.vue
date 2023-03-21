<template>
  <div id="main">

    <div id="scene" style="width:1300px; height:600px">
      <ThreeScene />
    </div>

    <!-- <Transition name="slide"> -->
    <!-- <div v-if="show_plan" id="mainplan">
      <PlanView/>
    </div>
    <div class="icon" v-else>
      <font-awesome-icon icon="fa-solid fa-angles-right" @click="openPlan"/>
    </div> -->
    <!-- </Transition> -->

    <!-- <div id="control_logs">
    <div id="titleControl">
      <router-link :to="{ name: 'controls' }" class="tablink borderLink">
        Controls
      </router-link>
      <router-link :to="{ name: 'logs' }" class="tablink borderLink">
        Logs
      </router-link>
      <div id="save" class="tooltip">
        <font-awesome-icon
          icon="fa-solid fa-save"
          style="font-size: 2.5em"
          @click="saveLogs()"
        />
        <div class="tooltiptext">save logs</div>
      </div>
    </div>
      <router-view />
    </div> -->
  </div>
</template>

<script>
import { store } from "./js/store.js";
import ThreeScene from "./views/ThreeScene.vue";
import PlanView from "./views/PlanView.vue";

export default {
  name: "App",
  created() {
    this.emitter.on("close-plan", () => {
      this.show_plan = false;
    });
    this.emitter.off("close-plan", () => {});
  },
  mounted() {
    let result;
    // while (result == null || result == "") {
    //   result = prompt("Please enter your id:", "000");
    // }
    const log = {
      time_stamp: new Date().toISOString(),
      message: "Operator ID: " + result,
      color: "white",
      name: store.logs.length,
    };
    store.logs.push(log);
  },
  data() {
    return {
      show_plan: false,
      store,
    };
  },
  methods: {
    openPlan() {
      this.show_plan = true;
    },
    saveLogs() {
      this.emitter.emit("save-logs");
    },
  },
  components: {
    PlanView,
    ThreeScene,
},
};
</script>

<style>
@import url('https://fonts.googleapis.com/css2?family=Fira+Sans:wght@100;400&display=swap');
@import "./style/css/App.css";
</style>

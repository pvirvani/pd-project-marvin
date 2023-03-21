<template>
  <ThreeScene />
  <Controls />
</template>

<script>
import { store } from "./js/store.js";
import ThreeScene from "./views/ThreeScene.vue";
import Controls from "./views/ControlsView.vue";
import lego_map from "./assets/lego_map.json";
import model from "./assets/model.json";

export default {
  name: "App",
    created() {
        store.lego_map = lego_map;
        store.model = model;
  },
  mounted() {
    this.init();
  },
  data() {
    return {
      store,
    };
  },
  methods: {
    init() {
      window.addEventListener("keydown", this.onKeyDown);
    },

    onKeyDown(event) { 
      event.preventDefault();
      if (event.keyCode === 38) {
        
        store.place_enabled = false;
        if (store.picked_lego === null) {
          store.rotated = false;
          store.pick_enabled = true;
        }
      }
      else if (event.keyCode === 40) {
        store.pick_enabled = false;
        if (store.picked_lego !== null) {
          store.place_enabled = true;
        }
      }
      else if (event.keyCode === 37) { store.rotated = true; }
      else if (event.keyCode === 39) { store.rotated = false; }
    },
  },
  components: {
    ThreeScene,
    Controls,
  },
};
</script>

<style>
@import "./style/css/App.css";
</style>

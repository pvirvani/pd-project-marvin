<template>
  <div >
  <div id="container">
    <div id="arms" class="view">
      <div id="lefthand" :class="left_arm_style">
        <img src="../assets/left-hand.png" />
        <p>Left</p>
      </div>
      <div id="righthand" :class="right_arm_style">
        <img src="../assets/right-hand.png" />
        <p>Right</p>
      </div>
      <div id="simple" v-if="simple">
        <img src="../assets/simple.jpg" />
        <!-- <p>Simple</p> -->
      </div>
      <div id="complex" v-else-if="complex">
        <img src="../assets/complex.jpg" />
        <!-- <p>Complex</p> -->
      </div>
    </div>
    <SceneControls
      @rotate-cam="rotateCamera"
      @reset-cam="resetCamera"
      @zoom-cam="zoomCamera"
    />
  </div>
</div>
</template>

<script>
import SceneCreator from "../js/SceneCreator.js";
import { createBase, drawArrow } from "../js/ModelCreator.js";
import SceneControls from "../components/SceneControls.vue";
import VisionInterface from "../js/VisionInterface.js";
import { PickPlace } from "../js/SpecialEffects.js";
import { store } from "../js/store";

export default {
  name: "ThreeScene",
  data() {
    return {
      pick_pos: null,
      place_pos: null,
      action_id: null,
    };
  },
  created() {
    this.emitter.on("choose-action", (evt) => {
      this.pick_pos = evt.pick_pos;
      this.place_pos = evt.place_pos;
      this.action_id = evt.action_id;
      drawArrow(this.pick_pos, this.place_pos, this.scene);
    });
    this.emitter.off("choose-action", (evt) => {
      this.pick_pos = evt.pick_pos;
      this.place_pos = evt.place_pos;
      this.action_id = evt.action_id;
      drawArrow(this.pick_pos, this.place_pos, this.scene);
    });
    this.emitter.on("action-execution", (evt) => {
      new PickPlace(
        this.renderer,
        this.scene,
        this.camera,
        evt.lego_,
        evt.place_pos,
        evt.rot_
      );
    });
    this.emitter.off("action-execution", (evt) => {
      new PickPlace(
        this.renderer,
        this.scene,
        this.camera,
        evt.lego_,
        evt.place_pos,
        evt.rot_
      );
    });
  },
  mounted() {
    this.init();
  },
  methods: {
    init() {
      this.container = document.getElementById("container");
      const sceneCreator_ = new SceneCreator(this.container);
      this.scene = sceneCreator_.getScene();
      this.camera = sceneCreator_.getCamera();
      this.renderer = sceneCreator_.getRenderer();
      createBase(this.scene);

      const vision_iface_ = new VisionInterface();
      vision_iface_.init(this.renderer, this.scene, this.camera);
      vision_iface_.updateLegos();
    },
    resetCamera() {
      this.scene.rotation.set(0, 0, 0);
      this.camera.position.set(0.1, 10, 10);
    },
    rotateCamera(rx, ry) {
      let new_rot_x;
      let new_rot_y;
      let new_rot_z;
      new_rot_x = this.scene.rotation.x + rx;
      new_rot_y = this.scene.rotation.y + ry;
      new_rot_z = this.scene.rotation.z;
      this.scene.rotation.set(new_rot_x, new_rot_y, new_rot_z);
    },
    moveCamera(dx, dz) {
      let new_pos_x = this.camera.position.x + dx;
      let new_pos_y = this.camera.position.y;
      let new_pos_z = this.camera.position.z + dz;
      this.camera.position.set(new_pos_x, new_pos_y, new_pos_z);
    },
    zoomCamera(away = false) {
      let new_pos_x;
      let new_pos_y;
      let new_pos_z;
      if (!away) {
        new_pos_x = this.camera.position.x;
        new_pos_y = this.camera.position.y + 2.5;
        new_pos_z = this.camera.position.z + 1.25;
        this.camera.position.set(new_pos_x, new_pos_y, new_pos_z);
      } else {
        new_pos_x = this.camera.position.x;
        new_pos_y = this.camera.position.y - 2.5;
        new_pos_z = this.camera.position.z - 1.25;
        this.camera.position.set(new_pos_x, new_pos_y, new_pos_z);
      }
    },
  },
  computed: {
    left_arm_style() {
      if (store.robot_motion_status === true && store.arm === "left") {
        return "blink_bg";
      } else {
        return "normal_bg";
      }
    },
    right_arm_style() {
      if (store.robot_motion_status === true && store.arm === "right") {
        return "blink_bg";
      } else {
        return "normal_bg";
      }
    },
    simple() {
      return store.pattern === "simple";
    },
    complex() {
      return store.pattern === "complex";
    },
  },
  components: {
    SceneControls,
  },
};
</script>

<style>
@import "../style/css/ThreeScene.css";
</style>

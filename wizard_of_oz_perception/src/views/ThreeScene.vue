<template></template>
<script>
import { Vector2, Vector3, Raycaster, Group } from "three";
import SceneCreator from "../js/SceneCreator.js";
import { createBase } from "../js/ModelCreator.js";
import CreateLego from "../js/LegoCreator.js";
import { store } from "../js/store.js";

export default {
  name: "ThreeScene",
  data() {
    return {
      store,
    };
  },
  created() {
    this.emitter.on("pattern-selection", () => {
      this.visu_json();
    });
    this.emitter.off("pattern-selection", () => {
      this.visu_json();
    });
  },
  mounted() {
    this.init();
  },
  methods: {
    init() {
      /**
       * Static Assembly Corners Position
       */
      this.corners = {
        lego_101: {
          size: 2,
          position: { x: 1, y: 7, z: 0 },
          rotation: false,
          color: "g",
        },
        lego_102: {
          size: 2,
          position: { x: 1, y: 15, z: 0 },
          rotation: false,
          color: "g",
        },
        lego_103: {
          size: 2,
          position: { x: 7, y: 7, z: 0 },
          rotation: false,
          color: "g",
        },
        lego_104: {
          size: 2,
          position: { x: 7, y: 15, z: 0 },
          rotation: false,
          color: "g",
        },
      };
      this.objects = [];
      /**
       * Create the scene
       */
      this.sceneCreator_ = new SceneCreator();
      this.scene = this.sceneCreator_.getScene();
      this.camera = this.sceneCreator_.getCamera();
      this.renderer = this.sceneCreator_.getRenderer();
      this.container = document.getElementById("scene");
      /**
       * Create the Fixed Components:
       * - Green Platform
       * - Green Lego Corners
       */
      createBase(this.scene);
      this.createCorners();
      /**
       * Pick & Place
       */
      this.container.addEventListener("click", this.click);
      this.container.addEventListener("dblclick", this.onDoubleClick, false);
      this.mouse = new Vector2();
      this.raycaster = new Raycaster();
    },

    createCorners() {
      let pos;
      for (const corner of Object.keys(this.corners)) {
        pos = new Vector3(
          this.corners[corner]["position"]["x"],
          this.corners[corner]["position"]["y"],
          this.corners[corner]["position"]["z"]
        );
        new CreateLego(
          this.scene,
          this.corners[corner]["size"],
          this.corners[corner]["color"],
          corner,
          pos,
          this.corners[corner]["rotation"]
        );
      }
    },

    visu_json() {
      this.scene.remove(this.assembly);
      this.assembly = new Group();
      this.assembly.name = "assembly";
      for (let lego in store.lego_map[store.pattern]) {
        let size = store.lego_map[store.pattern][lego].length;
        let color = lego[0];
        let rot_ = false;
        let p_xx_yy = store.lego_map[store.pattern][lego][0].split("_");
        let pos = {};
        pos.x = Number(p_xx_yy[1]);
        pos.y = Number(p_xx_yy[2]);

        if (size === 4) {
          rot_ =
            store.lego_map[store.pattern][lego][
              store.lego_map[store.pattern][lego].length - 1
            ];
          pos.z = Number(store.lego_map[store.pattern][lego][2]) - 1;
        } else {
          pos.z = Number(store.lego_map[store.pattern][lego][1]) - 1;
        }
        new CreateLego(this.assembly, size, color, lego, pos, rot_);
        let lego_obj = this.assembly.getObjectByName(lego, true);
        this.objects.push(lego_obj);
      }
      this.scene.add(this.assembly);
    },

    click(event) {
      event.preventDefault();
      if (store.pick_enabled) {
        this.pick(event);
      } else if (store.place_enabled) {
        this.place(event);
      }
    },

    pick(event) {
      this.mouse.x = (event.clientX / this.container.clientWidth) * 2 - 1;
      this.mouse.y = -(event.clientY / this.container.clientHeight) * 2 + 1;
      this.raycaster.setFromCamera(this.mouse, this.camera);

      const intersections = this.raycaster.intersectObjects(this.objects, true);

      if (intersections.length > 0) {
        const object = intersections[0].object;

        store.picked_lego = object.parent.parent;
        // style change
        let pick = document.getElementById("pick-enabled");
        pick.innerHTML = "Picked : " + object.parent.parent.name;
        pick.style.color = "#000000";
        pick.style.backgroundColor = "rgb(0, 255, 0)";

        store.pick_enabled = false;
        this.objects.splice(this.objects.indexOf(object.parent.parent), 1);
        this.assembly.remove(object.parent.parent);
        /**
         * if Pick from Assembly Zone
         */
        this.updateModel();
      }
      this.renderer.render(this.scene, this.camera);
    },

    updateModel() {
      let p_xx_yy =
        "p_" +
        this.store.picked_lego.userData.position.x.toLocaleString("en-US", {
          minimumIntegerDigits: 2,
        }) +
        "_" +
        this.store.picked_lego.userData.position.y.toLocaleString("en-US", {
          minimumIntegerDigits: 2,
        });
      let p_xx_yy2;
      if (this.store.picked_lego.userData.rotated) {
        p_xx_yy2 =
          "p_" +
          (this.store.picked_lego.userData.position.x + 1).toLocaleString(
            "en-US",
            { minimumIntegerDigits: 2 }
          ) +
          "_" +
          this.store.picked_lego.userData.position.y.toLocaleString("en-US", {
            minimumIntegerDigits: 2,
          });
      } else {
        p_xx_yy2 =
          "p_" +
          this.store.picked_lego.userData.position.x.toLocaleString("en-US", {
            minimumIntegerDigits: 2,
          }) +
          "_" +
          (this.store.picked_lego.userData.position.y + 1).toLocaleString(
            "en-US",
            { minimumIntegerDigits: 2 }
          );
      }

      let z;
      if (
        p_xx_yy in store.model &&
        !Object.keys(JSON.parse(JSON.stringify(store.model))[p_xx_yy]).length ==
          0
      ) {
        z = Object.keys(
          JSON.parse(JSON.stringify(store.model))[p_xx_yy]
        ).length;
        delete store.model[p_xx_yy][z];
      }
      if (
        p_xx_yy2 in store.model &&
        !Object.keys(JSON.parse(JSON.stringify(store.model))[p_xx_yy2])
          .length == 0
      ) {
        delete store.model[p_xx_yy2][z];
      }
    },

    place(event) {
      this.y = Math.floor(
        (Math.round(
          1000 * (event.clientX / this.container.clientWidth) * 2 - 1
        ) -
          520) /
          41
      );
      this.x = Math.floor(
        (Math.round(
          1000 * (event.clientY / this.container.clientHeight) * 2 + 1
        ) -
          560) /
          120
      );
      let p_xx_yy =
        "p_" +
        this.x.toLocaleString("en-US", { minimumIntegerDigits: 2 }) +
        "_" +
        this.y.toLocaleString("en-US", { minimumIntegerDigits: 2 });
      this.z = 0;

      if (p_xx_yy in store.model) {
        this.z = Object.keys(
          JSON.parse(JSON.stringify(store.model))[p_xx_yy]
        ).length;
      }
      let pos_ = new Vector3(this.x, this.y, this.z);
      new CreateLego(
        this.assembly,
        store.picked_lego.userData.size,
        store.picked_lego.name[0],
        store.picked_lego.name,
        pos_,
        store.rotated
      );
      let lego_obj = this.assembly.getObjectByName(
        store.picked_lego.name,
        true
      );
      this.objects.push(lego_obj);
      store.place_enabled = false;
      this.updateModels(
        store.picked_lego.name,
        { x: this.x, y: this.y, z: this.z + 1 },
        store.rotated
      );
      store.picked_lego = null;

      let pick = document.getElementById("pick");
      pick.innerHTML = "Pick";
      pick.style.color = "";
      pick.style.backgroundColor = "";
    },

    updateModels(name, pos, rotation) {
      let p_xx_yy =
        "p_" +
        this.x.toLocaleString("en-US", { minimumIntegerDigits: 2 }) +
        "_" +
        this.y.toLocaleString("en-US", { minimumIntegerDigits: 2 });
      let p_xx_yy2;

      if (name[1] === "b") {
        if (this.store.picked_lego.userData.rotated) {
          p_xx_yy2 =
            "p_" +
            (this.x + 1).toLocaleString("en-US", { minimumIntegerDigits: 2 }) +
            "_" +
            this.y.toLocaleString("en-US", { minimumIntegerDigits: 2 });
        } else {
          p_xx_yy2 =
            "p_" +
            this.x.toLocaleString("en-US", { minimumIntegerDigits: 2 }) +
            "_" +
            (this.y + 1).toLocaleString("en-US", { minimumIntegerDigits: 2 });
        }
      }

      /* Update Model */
      if (p_xx_yy in store.model) {
        store.model[p_xx_yy][pos.z] = name[0];
      }
      if (name[1] === "b" && p_xx_yy2 in store.model) {
        store.model[p_xx_yy2][pos.z] = name[0];
      }
      /* Update Lego Map */
      if (name[1] === "c") {
        store.lego_map[store.pattern][name][0] = p_xx_yy;
        store.lego_map[store.pattern][name][1] = pos.z;
      } else {
        store.lego_map[store.pattern][name][0] = p_xx_yy;
        store.lego_map[store.pattern][name][1] = p_xx_yy2;
        store.lego_map[store.pattern][name][2] = pos.z;
        store.lego_map[store.pattern][name][3] = rotation;
      }
      console.log(store.model);
      console.log(store.lego_map);
    },

    onDoubleClick(event) {
      event.preventDefault();

      this.y = Math.floor(
        (Math.round(
          1000 * (event.clientX / this.container.clientWidth) * 2 - 1
        ) -
          520) /
          41
      );
      this.x = Math.floor(
        (Math.round(
          1000 * (event.clientY / this.container.clientHeight) * 2 + 1
        ) -
          560) /
          120
      );
      let p_xx_yy =
        "p_" +
        this.x.toLocaleString("en-US", { minimumIntegerDigits: 2 }) +
        "_" +
        this.y.toLocaleString("en-US", { minimumIntegerDigits: 2 });

      if (
        store.model.p_xx_yy &&
        !Object.keys(store.model[p_xx_yy]).length === 0
      ) {
        this.z = store.model[p_xx_yy].length;
      } else {
        this.z = 0;
      }
      alert(
        "double ( x: " + this.x + ", y: " + this.y + ", z: " + this.z + " )"
      );
    },
  },
};
</script>
<style>
@import "../style/css/ThreeScene.css";
</style>

<!-- 
    StartStopButton.vue 
    ---
    @author: Lukas Loiodice.
    LIG/Marvin France, 2022. 
    ---
    @modified by: Belal HMEDAN.
    LIG/Marvin France, 2022.
    ---
-->
<template>
  <div id="startStop" class="startStop" @click="startStop()">
    <div class="fondu" x="0" y="0" width="200" height="200"></div>
    <div class="icone" width="200" height="200">
      <div
        class="depart animation"
        x="0"
        y="0"
        width="200"
        height="200"
        fill="#fff"
      ></div>
      <div
        class="depart derecha"
        x="0"
        y="0"
        width="200"
        height="200"
        fill="#fff"
      ></div>
    </div>
    <div class="puntero tooltip">
      <div class="tooltiptext">Click me to stop/resume the execution</div>
    </div>
  </div>
</template>

<script lang="js">
import { store } from "../js/store.js";
import PlanScheduler from "../js/ExecutionScheduler.js";

export default {
    name: 'StartStopbutton',
    mounted() {
        if (store.robot_can_move=== true) {
            document.getElementById('startStop').classList.toggle('active');
        }
    },
    data() {
        return {
            store,
            plan_exe_: new PlanScheduler(this.emitter),
        }
    },
    methods: {
        startExecution() {
            /**
             * Send start signal
             */

            store.robot_can_move = true;
            this.plan_exe_.init();
            const log = {
                time_stamp: new Date().toISOString(),
                message: "Operation has been Started!",
                color: "white",
                name: store.logs.length
            }
            store.logs.push(log);

        },
        stopExecution() {
            /**
             * Send stop signal
             */
            document.getElementById('startStop').classList.toggle('active');
            store.robot_can_move = false;
            const log = {
                time_stamp: new Date().toISOString(),
                message: "Operation has been Stopped!",
                color: "orange",
                name: store.logs.length
            }
            store.logs.push(log);
        },
        startStop() {
            if (store.arm !== null && store.pattern !== null) {
                if (store.emergency === false) {
                    if (store.robot_can_move === false) {
                        document.getElementById('startStop').classList.toggle('active');
                        if (document.getElementById('startStop').classList[1] != undefined) {
                            this.startExecution();
                        }
                    }
                    else {
                            this.stopExecution();
                        }
                    }
                }
                else {
                    const log = {
                        time_stamp: new Date().toISOString(),
                        message: "Operation Can't be Started! arm: " + store.arm + " pattern: " + store.pattern,
                        color: "white",
                        name: store.logs.length
                    }
                    store.logs.push(log);
                }
        }
    },
    components: {
        PlanScheduler,
    }
    
}
</script>

<style>
@import "../style/css/StartStopButton.css";
</style>

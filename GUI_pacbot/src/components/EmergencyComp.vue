<!--     ---
    @author: Lukas Loiodice.
    LIG/Marvin France, 2022.
    ---
    @modified by: Belal HMEDAN.
    LIG/Marvin France, 2022.
    --- 
-->
<template>
    <div id = "stopbutton" class ="tooltip">
        <div class="boulon" id="boulon_hg"></div>
        <div class="boulon" id="boulon_hd"></div>
        <div class="boulon" id="boulon_bg"></div>
        <div class="boulon" id="boulon_bd"></div>
        <p id = "emergencytop">EMERGENCY STOP</p>
        <p id = "emergencybot">EMERGENCY STOP</p>
        <div class="bigcercle tooltip" @click="stopExecution()">
            <div class="tooltiptext"> 
                Click me to interrupt the execution 
            </div>
        </div>
    </div>
</template>

<script>
import { store } from "../js/store.js";

export default {
    name: "EmergencyComp",
    data() { 
        return {
            store,
        }
    },
    methods: {
        stopExecution() {
            console.log(store.robot_can_move);
            if (store.robot_can_move=== true) { 
                document.getElementById('startStop').classList.toggle('active');
            }
            store.robot_can_move = false;
            store.emergency = true;
            const log = {
                time_stamp: new Date().toISOString(),
                message: "Emergency Stop Was Pressed!",
                color: "red",
                name: store.logs.length
            }
            store.logs.push(log);
            console.log("emergency Pressed!");
        },
    }
}
</script>

<style>
    @import '../style/css/EmergencyComp.css';
</style>
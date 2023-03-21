<template>
    <div class="plan">
    <div id="plan-header">
        <h1>
            Plan
        </h1>
        <font-awesome-icon icon="fa-solid fa-x" @click="closePlan"/>
    </div>
    <div class="planlist">
        <ul id="planOl">
        <Action
        v-for="(action, index) in store.plan"
            :action_id="index"
            :actor="action.actor"
            :color="action.color"
            :size="action.size"
            :lego_name="action.lego"
            :pick_pos="action.pick"
            :place_pos="action.place"
            :blocked="action.blocked"
            :status="action.status"
            :key="action.act_id"
            @fix="removeStep"
            @error="addLogMessage(`Can't see the execution of pass step`, `orange`)"
        />
        </ul>
    </div>
</div>
</template>

<script>
import Action from '../components/ActionComp.vue';
import PlanningInterface from "../js/PlanningInterface.js";
import { store } from '../js/store.js'

export default {
    name: 'PlanView',
    mounted() { 
        this.init();
    }, 
    data() {
        return {
            plan_iface_: new PlanningInterface(),
            store,
        }
    },
    methods: {
        init() { 
            this.logs = document.getElementById('logs', true);
        },

        removeStep(event) {
            store.plan[event.act_id].actor = "operator";
            store.plan[event.act_id].changed = true;
        },

        addLogMessage(message, color="white"){
            let log = {
                time_stamp: new Date().toISOString(),
                message: message,
                color: color,
                name: store.logs.length
            }          
            store.logs.push(log);
        },
        closePlan() {
            this.emitter.emit("close-plan");
        },
    },
    components: { Action }
}
</script>

<style>
    @import '../style/css/PlanView.css';
</style>
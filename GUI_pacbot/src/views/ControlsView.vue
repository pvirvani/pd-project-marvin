<template>
<div id="controls" class="param">
    <div class="control" id="combobox1">
        <ComboBox
        selector_title = "Pattern"
        selector_name = "pattern"
        :selector_options="['complex', 'simple']"
        selector_id="combo1"
        :option="store.pattern"
        ref="combo1"
        />
    </div>
    <div class="control" id="combobox2">
        <ComboBox
        selector_title = "Hand"
        selector_name = "hand"
        :selector_options="['left', 'right']"
        selector_id="combo2"
        :option="store.arm"
        ref="combo2"
        />
    </div>
    <div class="control" id="speedslider">
        <SpeedSlider/>
    </div>
    <div class="control" id="startstop">
        <StartStopButton ref="ss_button"/>
    </div>
    <!-- <div class="control" id="crono">
        <CronoMeter ref="crono"/>
    </div> -->
    <div class = control id="emergency">
        <EmergencyComp/>
    </div>
</div>
</template>

<script>
import { store } from '../js/store.js';
import ComboBox from '../components/ComboBox.vue';
import StartStopButton from '../components/StartStopButton.vue';
import SpeedSlider from '../components/SpeedSlider.vue';
import EmergencyComp from '../components/EmergencyComp.vue';
// import CronoMeter from '../components/CronoMeter.vue'

export default {
    name: "ControlsView",
    data() {
        return {
        store,
    } },
    components: {
    ComboBox,
    StartStopButton,
    SpeedSlider,
    EmergencyComp
},
    created() {
        this.emitter.on('lock-combobox', (event) => {
            if (event.id === "combo1") { 
                this.$refs.combo1.disabled = true;
                store.pattern = event.value;
            }
            else if (event.id === "combo2") { 
                this.$refs.combo2.disabled = true;
                store.arm = event.value;
            }
            
        });
        this.emitter.off('lock-combobox', (event) => {
            if (event.id === "combo1") { 
                this.$refs.combo1.disabled = true;
                store.pattern = event.value;
            }
            else if (event.id === "combo2") { 
                this.$refs.combo2.disabled = true;
                store.arm = event.value;
            }
            
        });
    },
}
</script>

<style>
    @import '../style/css/ControlsView.css';
</style>
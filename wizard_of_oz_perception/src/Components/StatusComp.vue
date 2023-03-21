<template>
    <div id="status">
      <h2 :id="pick">Pick</h2>
      <h2 :id="place">Place</h2>
      <h2 :id="rotate">Rotated</h2>
    </div>
</template>
<script>
import { store } from '../js/store.js';

export default {
    name: 'Status',
    computed: { 
        pick(){
            return store.pick_enabled ? 'pick-enabled' : 'pick';
        },

        place() { 
            return store.place_enabled ? 'place-enabled' : 'place';
        },

        rotate() {
            return store.rotated ? 'rotate-enabled' : 'rotate';
        },
    },

}
</script>
<style>
    @import '../style/css/StatusComp.css';
</style>
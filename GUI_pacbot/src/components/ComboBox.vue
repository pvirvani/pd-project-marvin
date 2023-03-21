<!-- 
        ---
    @author: Belal HMEDAN.
    LIG/Marvin France, 2022.
    ---
 -->
<template>
  <div class="combobox">
    <label for="{{selector_name}}">
      <p>{{ selector_title }}</p>
    </label>
    <select
      :name="selector_name"
      :id="selector_id"
      :disabled="disabled"
      @change="lockCombo"
      v-if="!option"
    >
      <option selected value disabled>--select--</option>
      <option v-for="(opt, index) in selector_options" :key="`opt ${index}`">
        {{ opt }}
      </option>
    </select>
    <select :name="selector_name" :id="selector_id" disabled="true" v-else>
      <option selected value disabled>{{ option }}</option>
    </select>
  </div>
</template>

<script lang="js">
import { store } from "../js/store.js";

export default {
  name: "ComboBox",
  data() {
    return {
      disabled: false,
    };
  },
  props: {
    selector_title: {
      type: String,
      required: true,
    },
    selector_name: {
      type: String,
      required: true,
    },
    selector_id: {
      type: String,
      default: "combobox",
    },
    selector_options: {
      type: Array,
      required: true,
    },
    option: {
      type: String,
      //required: true
    },
  },
  methods: {
    lockCombo(event) {
      this.emitter.emit("lock-combobox", {
        id: this.selector_id,
        value: event.target.value,
      });
      const log = {
        time_stamp: new Date().toISOString(),
        message: "ComboBox Choice [" + event.target.value + "] Was selected!",
        color: "orange",
        name: store.logs.length,
      };
      store.logs.push(log);
    },
  },
};
</script>

<style>
@import "../style/css/ComboBox.css";
</style>

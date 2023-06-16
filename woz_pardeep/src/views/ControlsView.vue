<template>
  <div id="controls">

    <button @click="publish" id="start" :style="{ 'border-color': border_color }">Publish Topic</button>
    <button @click="stopPublish" id="stop" :style="{'border-color':border_color}">Stop Publishing Topic</button>

    <ComboBox
      selector_title="Pattern"
      selector_name="pattern"
      :selector_options="['complex', 'simple']"
      selector_id="combo1"
      :option="store.pattern"
    />

    <Status />
    
  </div>
</template>
<script>
import ROSInterface from "../js/ROSInterface.js";
import ComboBox from "../Components/ComboBox.vue";
import Status from "../Components/StatusComp.vue";
import {store} from '../js/store.js';

export default {
  name: "Controls",
  data() {
    return {
        interval: null,
      store,
        border_color: '#C0C0C0',
      };
  },
  mounted() {
    this.init();
  },
  components: {
    ComboBox,
    Status,
  },
  methods: {
    init() {
      this.iface_ = new ROSInterface();
      this.model_topic = this.iface_.createTopic('/model', 'std_msgs/String');
      this.lego_map_topic = this.iface_.createTopic('/lego_map', 'std_msgs/String');
      this.rate = 10; // 10hz
      this.isPublishing = false;
    },

    publish() {
      if (!this.isPublishing) {
        this.interval = setInterval(this.publish_topics, 1000 / this.rate);
        this.isPublishing = true;
        this.border_color = '#f52f0f';
      }
    },

    publish_topics() {
      // model
      let model_msg_ = this.iface_.createMessage({ data: JSON.stringify(store.model) });
      this.model_topic.publish(model_msg_);
      // lego_map
      let lego_map_msg_ = this.iface_.createMessage({ data: JSON.stringify(store.lego_map) });
      this.lego_map_topic.publish(lego_map_msg_);
    },

    stopPublish() {
      if (this.isPublishing) {
        clearInterval(this.interval);
        this.isPublishing = false;
        this.border_color = '#42ff33';
      }
    },
  },
};
</script>
<style>
@import "../style/css/ControlsView.css";
</style>

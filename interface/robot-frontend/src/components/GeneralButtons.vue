<template>
<div class=" q-pa-md q-gutter-y-md column items-start flex"
  style="float: left; background-color:antiquewhite">
  <q-btn to="/"
      color="primary"
      push
      no-caps
      label="Projects"
      icon="o_home"
      size='40px'
      style="width:200px;height: 80px; font-size:20px" />
<!-- <q-btn to="/demo"
      color="primary"
      push
      no-caps
      label="Demonstration"
      icon="o_precision_manufacturing"
      size='40px'
      style="width:200px;height: 80px; font-size:20px" /> -->
      <!-- :to="{ name: 'demo', params: { id: global_project_id } } -->
<q-btn
      :to="{ name: 'demo', params: { id: gid } }"
      color="primary"
      push
      no-caps
      label="Demonstration"
      icon="o_precision_manufacturing"
      size='40px'
      style="width:200px;height: 80px; font-size:20px" />
<!-- <q-btn to="/actions"
      color="primary"
      push
      no-caps
      label="Actions"
      icon="o_question_mark"
      size='40px'
      style="width:200px;height: 80px; font-size:20px" /> -->
<q-btn
      :to="{ name: 'problems', params: { id: gid } }"
      color="primary"
      push
      no-caps
      label="Problems"
      icon="o_rule"
      size='40px'
      style="width:200px;height: 80px; font-size:20px" />
<br />
<br />
<br />
<br />
<br />
<br />
<br />
<br />
<br />
<br />
<br />
<br />
<br />
<!-- <br />
<br />
<br />
<br />
<br />
<br /> -->
<q-btn to="/newdomain"
      color="primary"
      push
      label="Domain"
      icon="o_domain"
      size='40px'
      style="width:200px;height: 50px; font-size:20px" />
</div>
</template>
<script setup>
import { global_pid } from "src/stores/global_pid";
let gid_store = global_pid();
let gid = gid_store.getgid()

</script>
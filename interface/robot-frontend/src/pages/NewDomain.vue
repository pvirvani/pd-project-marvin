<template>
  <div class="q-pa-md row items-start q-gutter-md">
  <GeneralButtons/>
  <q-card class="my-card bg-secondary text-white">
    <q-card-section>
      <div class="text-h6">Domain for Project: {{ parent_name }}</div>
    </q-card-section>
    <q-separator dark />
    <q-card-section>
      <pre>{{ domain }}</pre>
    </q-card-section>
    </q-card>
    </div>
</template>

<script setup>
import { ref } from "vue";
import GeneralButtons from "src/components/GeneralButtons.vue";
// import { api } from "../boot/axios";
import { useQuasar } from "quasar";
// import RosConnection from "src/pages/RosConnection.vue";
// import { action_parameter } from "src/stores/action_parameter";
import { global_pid } from "src/stores/global_pid";

let gpidstore = global_pid()

let domain = gpidstore.getdomain();


</script>

<style>
pre{
  white-space: pre-wrap;
  padding: 10px;
  width: 1200px;
}
</style>
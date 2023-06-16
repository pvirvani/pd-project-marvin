<template>
  <div class="q-pa-md q-gutter-y-md column" style="background-color: bisque">
    <p style="font-size: 20px; text-align: center">
      <b> Project: {{ parent_name }}</b>
    </p>
  </div>
  <div class="q-pa-md q-gutter-sm row">
    <div>
      <GeneralButtons />
    </div>
    <div class="q-pa-md q-gutter-sm" style="width: 600px">
      <q-table
        title="Problems"
        :columns="columns"
        :rows="problems"
        row-key="problem_id"
        selection="single"
        v-model:selected="selectedProblem"
      >
      </q-table>
      <!-- functionality of ADD Button -->
      <q-btn
        icon="o_add"
        color="primary"
        label="Add"
        @click="addprompt = true"
      />

      <!-- Fucntionality of Open Button -->
      <q-btn
        icon="o_file_open"
        color="primary"
        label="Open"
        @click="
          displayActions = true;
          getActions(parent_id, parent_name, selectedDemo[0].demo_id);
        "
      />

      <!-- Functionality of Clone Button -->
      <q-btn
        color="primary"
        icon="o_file_copy"
        label="Copy"
        @click="copyDemo(parent_id, parent_name, selectedDemo[0].demo_name)"
      />

      <!-- Functionality of Clone Button -->
      <!-- <q-btn color="primary"
               icon="o_drive_file_rename_outline"
               label="Edit"
               @click="editSelected(selected)" /> -->

      <q-btn
        color="primary"
        icon="o_drive_file_rename_outline"
        label="Edit"
        @click="editprompt = true"
      />

      <!-- Functionality of Delete Button -->
      <q-btn
        color="red"
        icon="o_delete"
        label="Delete"
        @click="deleteprompt = true"
        type="submit"
      />

      <q-dialog v-model="addprompt" persistent auto-close="false">
        <q-card style="min-width: 350px">
          <q-card-section>
            <div class="text-h6">Problem Data</div>
          </q-card-section>

          <q-card-section class="q-pt-none">
            <q-form class="q-gutter-md">
              <q-input
                filled
                v-model="p_name"
                type="text"
                label="Problem Name *"
                hint="problem1"
                lazy-rules
                :rules="[
                  (val) => (val && val.length > 0) || 'Please type something',
                ]"
              />
            </q-form>
          </q-card-section>

          <q-card-actions align="right" class="text-primary">
            <q-btn flat label="Cancel" v-close-popup />
            <q-btn
              flat
              label="OK"
              v-close-popup
              type="submit"
              @click="
                addProblem(parent_id, parent_name, p_name);
                initForm();
              "
            />
            <!-- @click="addProject(pname, pproblems, pactions)" -->
            <!-- <q-btn flat label="Reset" type="reset" @click="initForm()" /> -->
          </q-card-actions>
        </q-card>
      </q-dialog>

    </div>
    </div>
</template>

<script setup>
import { ref } from "vue";
import GeneralButtons from "src/components/GeneralButtons.vue";
import { api } from "../boot/axios";
import { useQuasar } from "quasar";
import RosConnection from "src/pages/RosConnection.vue";
import { action_parameter } from "src/stores/action_parameter";
import { global_pid } from "src/stores/global_pid";

const $q = useQuasar();

// const route = useRoute();
// const parent_id = route.params.id;
// let parent_name = ref("");
let problem_name = ref("");
let addprompt = ref(false);
let deleteprompt = ref(false);
let editprompt = ref(false);

let problems = ref([]);
let selectedProblem = ref([]);
// let displayActions = ref(false);
let gid_store = global_pid();
let parent_name = gid_store.getgname()
let parent_id = gid_store.getgid()
let p_name = ref("")

const columns = [
  // {
  //     name: 'id',
  //     label: 'ProjectId',
  //     // required: true,
  //     align: 'left',
  //     field: row => row.id,
  //     format: val => `${val}`,
  //     sortable: true
  // },
  {
    name: "problem_name",
    label: "Problem Name",
    required: true,
    align: "center",
    // field: (row) => row.demo_name,
    // format: (val) => `${val}`,
    // sortable: true,
  },
  // {
  //   name: "problem_solved",
  //   label: "Solved",
  //   required: true,
  //   align: "center",
  //   // field: (row) => row.demo_actions,
  //   // format: (val) => `${val}`,
  //   // sortable: true,
  // },
  // {
  //   name: "p_actions",
  //   label: "Actions",
  //   required: true,
  //   align: "center",
  //   field: (row) => row.p_actions,
  //   format: (val) => `${val}`,
  //   sortable: true,
  // },
];

function addProblem(pt_id, pt_name, p_name) {
  const payload = {
    parent_id: pt_id,
    parent_name: pt_name,
    p_name: p_name,
  };
  api
    .post("/adddemo", payload)
    .then((response) => {
      demos.value = response.data.demos;
      // books.value = response.data.books
      $q.notify({
        color: "green",
        position: "top",
        message: "Demo Created",
        icon: "o_done",
        timeout: 500,
      });
    })
    .catch(() => {
      $q.notify({
        color: "negative",
        position: "top",
        message: "Demo not created",
        icon: "report_problem",
        timeout: 500,
      });
    });
}

</script>
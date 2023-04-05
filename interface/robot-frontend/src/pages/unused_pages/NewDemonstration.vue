// This page combines the previous OnDemonstration and OnActions Functionality
into one.

<template>
  <div class="q-pa-md q-gutter-y-md column" style="background-color: bisque">
    <p style="font-size: 20px; text-align: center">
      <b> Project: {{ parent_name }}</b>
    </p>
  </div>
  <br />
  <br />
  <div class="q-pa-md q-gutter-sm row">
    <div>
      <GeneralButtons />
    </div>
    <div class="q-pa-md q-gutter-sm" style="width: 600px">
      <q-table
        title="Demonstrations"
        :columns="columns"
        :rows="demos"
        row-key="demo_id"
        selection="single"
        v-model:selected="selectedDemo"
      >
      </q-table>
      <!-- functionality of ADD Button -->
      <q-btn
        icon="o_file_open"
        color="primary"
        label="Add"
        @click="addprompt = true"
      />

      <!-- functionality of Open Button -->
      <q-btn
        :to="{ name: 'demo', params: { id: global_project_id } }"
        icon="o_file_open"
        color="primary"
        label="Open"
      />

      <!-- Functionality of Copy Button -->
      <q-btn
        color="primary"
        icon="o_copy"
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
      <div class="q-pa-md q-gutter-md row">
        <q-card class="my-card" style="width: 700px">
          <q-card-section>
            <div class="text-h6">Selected Actions</div>
            <div class="text-subtitle2"></div>
          </q-card-section>
        </q-card>

        <q-card class="my-card" style="width: 700px">
          <q-card-section>
            <div class="text-h6">Avaialable Actions</div>
            <div class="text-subtitle2"></div>
          </q-card-section>
          <q-separator />
          <q-card-section>
            <q-table
              title="Actions"
              :columns="action_columns"
              :rows="actions"
              row-key="action_id"
              selection="single"
              v-model:selected="selectedAction"
            >
            </q-table>
          </q-card-section>
        </q-card>
      </div>

      <!-- add prompt for flask api -->
      <q-dialog v-model="addprompt" persistent auto-close="false">
        <q-card style="min-width: 350px">
          <q-card-section>
            <div class="text-h6">Demonstration Data</div>
          </q-card-section>

          <q-card-section class="q-pt-none">
            <q-form class="q-gutter-md">
              <q-input
                filled
                v-model="d_name"
                type="text"
                label="Demonstration Name *"
                hint="demo1"
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
                addDemo(parent_id, parent_name, d_name);
                initForm();
              "
            />
            <!-- @click="addProject(pname, pproblems, pactions)" -->
            <!-- <q-btn flat label="Reset" type="reset" @click="initForm()" /> -->
          </q-card-actions>
        </q-card>
      </q-dialog>
      <q-dialog v-model="editprompt" persistent auto-close="false">
        <q-card style="min-width: 350px">
          <q-card-section>
            <div class="text-h6">Demonstration Data</div>
          </q-card-section>

          <q-card-section class="q-pt-none">
            <q-form class="q-gutter-md">
              <q-input
                filled
                v-model="d_name"
                type="text"
                label="Demonstration Name *"
                hint="demo1"
                lazy-rules
                :rules="[
                  (val) => (val && val.length > 0) || 'Please type something',
                ]"
              />

              <!-- <q-input filled
                                   type="text"
                                   v-model="pproblems"
                                   label="Available Problems *"
                                   lazy-rules
                                   :rules="[val => val && val.length > 0 || 'Please type something']" />
                          <q-input filled
                                   type="text"
                                   v-model="pactions"
                                   label="Available Actions *"
                                   lazy-rules
                                   :rules="[val => val && val.length > 0 || 'Please type something']" /> -->
              <!-- <q-checkbox v-model="bread"
                                      label="Read" /> -->

              <!-- <q-toggle v-model="accept"
                                    label="I accept the license and terms" /> -->

              <div>
                <!-- <q-btn label="Submit"
                                     type="submit"
                                     color="primary" />
                              <q-btn label="Reset"
                                     type="reset"
                                     color="primary"
                                     flat
                                     class="q-ml-sm" /> -->
              </div>
            </q-form>
          </q-card-section>

          <q-card-actions align="right" class="text-primary">
            <q-btn flat label="Cancel" v-close-popup />
            <q-btn
              flat
              label="Update"
              v-close-popup
              type="submit"
              @click="
                editDemo(
                  parent_id,
                  parent_name,
                  selectedDemo[0].demo_id,
                  d_name
                );
                initForm();
              "
            />
            <!-- @click="editProject(pname, pproblems, pactions, selectedProject[0].p_id)" -->
            <!-- <q-btn flat label="Reset" type="reset" @click="initForm()" /> -->
          </q-card-actions>
        </q-card>
      </q-dialog>
      <!-- delete prompt flask api -->
      <!-- edit prompt for flask api -->
      <q-dialog v-model="deleteprompt" persistent auto-close="false">
        <q-card style="min-width: 350px">
          <q-card-section>
            <div class="text-h6">
              Do you want to delete: {{ selectedDemo[0].demo_name }}
            </div>
            <!-- <div> {{}}</div> -->
          </q-card-section>
          <q-card-actions
            align="right"
            class="text-primary"
            label="Do you want to delete the Project"
          >
            <q-btn flat label="Cancel" v-close-popup />
            <q-btn
              flat
              label="Delete"
              v-close-popup
              type="submit"
              @click="
                deleteDemo(parent_id, parent_name, selectedDemo[0].demo_id)
              "
            />
          </q-card-actions>
        </q-card>
      </q-dialog>
    </div>
  </div>
</template>

<script setup>
import { ref } from "vue";
import GeneralButtons from "src/components/GeneralButtons.vue";
import { useRoute } from "vue-router";
import { api } from "../boot/axios";
import { useQuasar } from "quasar";

const $q = useQuasar();

const route = useRoute();
const parent_id = route.params.id;
let parent_name = ref("");
let d_name = ref("");
let addprompt = ref(false);
let deleteprompt = ref(false);
let editprompt = ref(false);

let demos = ref([]);
let selectedDemo = ref([]);

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
    name: "demo_name",
    label: "Demonstration Name",
    required: true,
    align: "center",
    field: (row) => row.demo_name,
    format: (val) => `${val}`,
    sortable: true,
  },
  {
    name: "demo_actions",
    label: "Actions",
    required: true,
    align: "center",
    field: (row) => row.demo_actions,
    format: (val) => `${val}`,
    sortable: true,
  },
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

function getProjectData(pt_id) {
  const payload = {
    parent_id: pt_id,
  };
  api
    .post("/getprojectdata", payload)
    .then((response) => {
      parent_name.value = response.data.projdata;
      getDemos(pt_id, parent_name.value);
      // books.value = response.data.books
      $q.notify({
        color: "green",
        position: "top",
        message: "Project Opened",
        icon: "o_done",
        timeout: 500,
      });
    })
    .catch(() => {
      $q.notify({
        color: "negative",
        position: "top",
        message: "Project Loading Failed: Server Unavailable",
        icon: "report_problem",
        timeout: 500,
      });
    });
}

function getDemos(pt_id, pt_name) {
  const payload = {
    parent_id: pt_id,
    parent_name: pt_name,
  };
  api
    .post("/getdemos", payload)
    .then((response) => {
      demos.value = response.data.demos;
      // books.value = response.data.books
      $q.notify({
        color: "green",
        position: "top",
        message: "Demo Retrieved",
        icon: "o_done",
        timeout: 500,
      });
    })
    .catch(() => {
      $q.notify({
        color: "negative",
        position: "top",
        message: "Project Loading Failed: Server Unavailable",
        icon: "report_problem",
        timeout: 500,
      });
    });
}

function addDemo(pt_id, pt_name, dname) {
  const payload = {
    parent_id: pt_id,
    parent_name: pt_name,
    d_name: dname,
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

function deleteDemo(pt_id, pt_name, dem_id) {
  const payload = {
    parent_id: pt_id,
    parent_name: pt_name,
    demo_id: dem_id,
  };
  api
    .post("/deletedemo", payload)
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

function copyDemo(pt_id, pt_name, dem_name) {
  const payload = {
    parent_id: pt_id,
    parent_name: pt_name,
    demo_name: dem_name,
  };
  api
    .post("/copydemo", payload)
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
function editDemo(pt_id, pt_name, dem_id, dem_name) {
  const payload = {
    parent_id: pt_id,
    parent_name: pt_name,
    demo_id: dem_id,
    demo_name: dem_name,
  };
  api
    .post("/updatedemo", payload)
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
let actions = ref([]);
const action_columns = [
  {
    name: "action_name",
    label: "Action Name",
    required: true,
    align: "center",
    field: (row) => row.action_name,
    format: (val) => `${val}`,
    sortable: true,
  },
  {
    name: "action_parameters",
    label: "Action Parameters",
    required: true,
    align: "center",
    field: (row) => row.action_parameters,
    format: (val) => `${val}`,
    sortable: true,
  },
];
function getActions() {
  api
    .get("/actions")
    .then((response) => {
      actions.value = response.data.actions;
    })
    .catch(() => {
      $q.notify({
        color: "negative",
        position: "top",
        message: "Loading failed",
        icon: "report_problem",
      });
    });
}

getProjectData(parent_id);
// getDemos(parent_id,parent_name)
getActions();
</script>

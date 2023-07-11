<template>
  <div class="q-pa-md q-gutter-y-md column" style="background-color: bisque">
    <p style="font-size: 20px; text-align: center">
      <b> Project: {{ parent_name }}</b>
    </p>
  </div>
  <!-- <br />
  <br /> -->
  <div class="q-pa-md q-gutter-sm row">
    <div>
      <GeneralButtons />
    </div>
    <div class="q-pa-md q-gutter-sm column">
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
      <!-- Functionality of Genearte Domain Button -->
      
        <q-btn
        color="green"
        icon="o_account_tree"
        label="Generate Domain"
        padding="6px 190px "
        @click="generateDomain(parent_id,parent_name);showDomain=true"
        type="submit"
      />
      
     
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
      <!-- edit prompt for demos -->
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
    <!--  -->
    <!-- ------------------- Displaying Actions ---------------------- -->
    <!--  -->

    <div
      class="q-pa-md q-gutter-sm"
      style="width: 600px"
      v-if="displayActions == true"
    >
      <!-- <DemoActions /> -->
      <!-- <div class="q-pa-md q-gutter-sm" style="width: 600px"> -->
      <q-table
        title="Actions"
        :columns="actioncolumns"
        :rows="cr_actions"
        row-key="action_id"
        selection="single"
        v-model:selected="selectedAction"
      >
        <!-- <template v-slot:top-right>
        <q-btn color="" disable label="{{selectedDemo[0].demo_id}}" @click="addRow" />
      </template> -->
        <!-- {{ selectedDemo[0].demo_id }} -->
      </q-table>
      <!-- <div>
      {{ cr_actions }}
      </div> -->
      <!-- functionality of ADD Button -->
      <q-btn
        icon="o_add"
        color="primary"
        label="Add"
        align="center"
        @click="addactionprompt = true"
      />
      <!-- Functionality of Edit Button -->
      <!-- <q-btn
        color="primary"
        icon="o_drive_file_rename_outline"
        label="Edit"
        @click="editactionprompt = true"
      /> -->

      <!-- Functionality of Delete Button -->
      <q-btn
        color="red"
        icon="o_delete"
        label="Delete"
        align="center"
        @click="deleteactionprompt = true"
        type="submit"
      />
      <!-- functionality of ADD Button -->
      <!-- <q-btn
        icon="o_play_arrow"
        color="green"
        label="Demonstrate"
        align="center"
        @click="demonstrateprompt = true"
      /> -->

      <q-btn
        icon="o_format_list_numbered"
        color="green"
        label="Generate Sequence"
        align="center"
        @click="
          generateSequence(parent_id, parent_name, selectedDemo[0].demo_id);
          showSequence=true
        "
      />
      <!-- Add Actions -->
      <q-dialog v-model="addactionprompt" persistent auto-close="false">
        <q-card style="min-width: 1200px">
          <q-card-section>
            <div class="text-h6">Action Data</div>
          </q-card-section>

          <!-- <q-card-section class="q-pt-none">
            <q-form class="q-gutter-md">
              <q-input
                filled
                v-model="aname"
                type="text"
                label="Action Name *"
                hint="put block"
                lazy-rules
                :rules="[
                  (val) => (val && val.length > 0) || 'Please type something',
                ]"
              />
            </q-form>
          </q-card-section> -->

          <q-card-section class="q-pt-none">
            <q-form class="q-gutter-md">
              <RosConnection />
            </q-form>
          </q-card-section>

          <q-card-section
            class="q-pt-none"
            v-if="astore.getparameter().length >= 1"
          >
            <q-form class="q-gutter-md">
              <q-field filled label="Action Parameter" stack-label>
                <template v-slot:control>
                  <div class="self-center full-width no-outline" tabindex="0">
                    {{ astore.getparameter()[0] }}
                  </div>
                </template>
              </q-field>
            </q-form>
          </q-card-section>

          <q-card-actions align="right" class="text-primary">
            <q-btn flat label="Cancel" v-close-popup />
            <q-btn
              flat
              label="OK"
              :disable="astore.getparameter().length < 1 || aname == ''"
              type="submit"
              @click="
                addAction(
                  parent_id,
                  parent_name,
                  selectedDemo[0].demo_id,
                  selectedDemo[0].demo_name,
                  aname,
                  astore.getparameter()[0]
                );
                initactionForm();
              "
              v-close-popup
            />
            <!-- @click="addProject(pname, pproblems, pactions)" -->
            <!-- <q-btn flat label="Reset" type="reset" @click="initForm()" /> -->
          </q-card-actions>
        </q-card>
      </q-dialog>

      <!-- Delete Action -->

      <!-- delete prompt flask api -->
      <q-dialog v-model="deleteactionprompt" persistent auto-close="false">
        <q-card style="min-width: 350px">
          <q-card-section>
            <div class="text-h6">
              Do you want to delete action: {{ selectedAction[0].action_name }}
            </div>
            <!-- <div> {{}}</div> -->
          </q-card-section>
          <q-card-actions
            align="right"
            class="text-primary"
            label="Do you want to delete the Action"
          >
            <q-btn flat label="Cancel" v-close-popup />
            <q-btn
              flat
              label="Delete"
              v-close-popup
              type="submit"
              @click="
                deleteAction(
                  parent_id,
                  parent_name,
                  selectedDemo[0].demo_id,
                  selectedAction[0].action_id
                )
              "
            />
          </q-card-actions>
        </q-card>
      </q-dialog>

      <!-- edit action -->
      <q-dialog v-model="editactionprompt" persistent auto-close="false">
        <q-card style="min-width: 350px">
          <q-card-section>
            <div class="text-h6">Action Data</div>
          </q-card-section>

          <q-card-section class="q-pt-none">
            <q-form class="q-gutter-md">
              <q-input
                filled
                v-model="aname"
                type="text"
                label="Action Name *"
                hint="action1"
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
                editAction(
                  parent_id,
                  parent_name,
                  selectedDemo[0].demo_id,
                  selectedAction[0].action_id,
                  aname
                );
                initForm();
              "
            />
            <!-- @click="editProject(pname, pproblems, pactions, selectedProject[0].p_id)" -->
            <!-- <q-btn flat label="Reset" type="reset" @click="initForm()" /> -->
          </q-card-actions>
        </q-card>
      </q-dialog>

      <!-- </div> -->
    </div>
    </div>
    <div
      class="q-pa-md q-gutter-sm"
      style="width: 600px"
      v-if="showSequence == true"
    >
    <div class="q-pa-md row items-start q-gutter-md">
    <q-card class="my-card bg-secondary text-white">
      <q-card-section>
        <div class="text-h6">Inferred Action Sequence</div>
      </q-card-section>
      <q-separator dark />
      <q-card-section>
        <ul>
        <li v-for="action in ac_sequence" :key="action">{{ action }}</li>
      </ul>
      </q-card-section>
      </q-card>
      </div>

      
    </div>

  </div>
</template>

<script setup>
import { ref } from "vue";
import GeneralButtons from "src/components/GeneralButtons.vue";
import DemoActions from "src/components/DemoActions.vue";
import { useRoute } from "vue-router";
import { api } from "../boot/axios";
import { useQuasar } from "quasar";
import RosConnection from "src/pages/RosConnection.vue";
import { action_parameter } from "src/stores/action_parameter";
import { global_pid } from "src/stores/global_pid";

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
let displayActions = ref(false);
let showSequence = ref(false)
let showDomain = ref(false)

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
    align: "left",
    field: (row) => row.demo_name,
    format: (val) => `${val}`,
    sortable: true,
  },
  {
    name: "demo_actions",
    label: "Actions",
    required: true,
    align: "left",
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

const actioncolumns = [
  // {
  //     name: 'id',
  //     label: 'ProjectId',
  //     // required: true,
  //     align: 'left',
  //     field: row => row.id,
  //     format: val => `${val}`,
  //     sortable: true
  // },
  // {
  //   name: "action_name",
  //   label: "Action Name",
  //   required: true,
  //   align: "left",
  //   field: (row) => row.action_name,
  //   format: (val) => `${val}`,
  //   sortable: true,
  // },

  {
    name: "actions",
    label: "Actions",
    required: true,
    align: "left",
    field: (row) => row.action_parameters,
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

// -------------------- for actions section -----------------
let addactionprompt = ref(false);
let deleteactionprompt = ref(false);
let editactionprompt = ref(false);
let selectedAction = ref([]);
// let actions = ref([]);
let cr_actions = ref([]);
// let aname = ref("");
const aname = "action";
let astore = action_parameter();
let gpidstore = global_pid();
let ac_sequence = ref([])
// let action_sequence = ref([])
let domain = ref('')
let domain_fomatted= ref('')

const action_columns = [
  {
    name: "action_name",
    label: "Action Name",
    required: true,
    align: "center",
    field: (row) => row.cr_actions.action_name,
    format: (val) => `${val}`,
    sortable: true,
  },
  {
    name: "action_parameters",
    label: "Action Parameters",
    required: true,
    align: "center",
    field: (row) => row.cr_actions.action_parameters,
    format: (val) => `${val}`,
    sortable: true,
  },
];
function getActions(pt_id, pt_name, pd_id) {
  const payload = {
    project_id: pt_id,
    project_name: pt_name,
    demo_id: pd_id,
  };
  api
    .post("/getactions", payload)
    .then((response) => {
      cr_actions.value = response.data.actions;
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

function getSequence(pt_id, pt_name, pd_id) {
  const payload = {
    project_id: pt_id,
    project_name: pt_name,
    demo_id: pd_id,
  };
  api
    .post("/getsequence", payload)
    .then((response) => {
      ac_sequence.value = response.data.action_sequence;
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

function addAction(pt_id, pt_name, pd_id, pd_name, aname, aparam) {
  const payload = {
    project_id: pt_id,
    project_name: pt_name,
    demo_id: pd_id,
    demo_name: pd_name,
    action_name: aname,
    action_parameters: aparam,
    // p_problems: 0,
    // p_actions: 0,
  };
  api
    .post("/addactions", payload)
    .then((response) => {
      getDemos(pt_id, pt_name);
      getActions(pt_id, pt_name, pd_id);
      astore.removeparameter();
      // books.value = response.data.books
      $q.notify({
        color: "green",
        position: "top",
        message: "Action Created",
        icon: "o_done",
        timeout: 500,
      });
    })
    .catch(() => {
      $q.notify({
        color: "negative",
        position: "top",
        message: "Loading failed",
        icon: "report_problem",
        timeout: 500,
      });
    });
}

function deleteAction(pt_id, pt_name, pd_id, act_id) {
  const payload = {
    parent_id: pt_id,
    parent_name: pt_name,
    demo_id: pd_id,
    action_id: act_id,
  };
  api
    .post("/deleteaction", payload)
    .then((response) => {
      // demos.value = response.data.demos;
      getActions(pt_id, pt_name, pd_id);
      // books.value = response.data.books
      $q.notify({
        color: "green",
        position: "top",
        message: "Action Deleted",
        icon: "o_done",
        timeout: 500,
      });
    })
    .catch(() => {
      $q.notify({
        color: "negative",
        position: "top",
        message: "Failed to Delete Action",
        icon: "report_problem",
        timeout: 500,
      });
    });
}

function editAction(pt_id, pt_name, pd_id, act_id, act_name) {
  const payload = {
    parent_id: pt_id,
    parent_name: pt_name,
    demo_id: pd_id,

    action_id: act_id,
    action_name: act_name,
  };
  api
    .post("/updateaction", payload)
    .then((response) => {
      // demos.value = response.data.demos;
      getActions(pt_id, pt_name, pd_id);
      // books.value = response.data.books
      $q.notify({
        color: "green",
        position: "top",
        message: "Action Updated",
        icon: "o_done",
        timeout: 500,
      });
    })
    .catch(() => {
      $q.notify({
        color: "negative",
        position: "top",
        message: "Failed to update action",
        icon: "report_problem",
        timeout: 500,
      });
    });
}

function generateDomain(pt_id, pt_name) {
  const payload = {
    parent_id: pt_id,
    parent_name: pt_name,
  };
  api
    .post("/generatedomain", payload)
    .then((response) => {
      domain.value = response.data.ppdl_content;
      gpidstore.setdomain(domain.value);
      $q.notify({
        color: "green",
        position: "top",
        message: "Domain Generated",
        icon: "o_done",
        timeout: 500,
      });
    })
    .catch(() => {
      $q.notify({
        color: "negative",
        position: "top",
        message: "Failed to Generate Domain",
        icon: "report_problem",
        timeout: 500,
      });
    });
}

function generateSequence(pt_id, pt_name, pd_id) {
  const payload = {
    parent_id: pt_id,
    parent_name: pt_name,
    demo_id: pd_id,
  };
  api
    .post("/generatesequence", payload)
    .then((response) => {
      getSequence(pt_id, pt_name, pd_id)
      $q.notify({
        color: "green",
        position: "top",
        message: "Sequence Generated",
        icon: "o_done",
        timeout: 500,
      });
    })
    .catch(() => {
      $q.notify({
        color: "negative",
        position: "top",
        message: "Failed to Generate Sequence",
        icon: "report_problem",
        timeout: 500,
      });
    });
}

function initForm() {
  d_name.value = "";
}

function initactionForm() {
  aname.value = "";
  aparam.value = "";
  // pactions.value = "";
}

getProjectData(parent_id)
// getDemos(parent_id,parent_name)
</script>

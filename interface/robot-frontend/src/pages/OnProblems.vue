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
          getActions(parent_id, parent_name, selectedProblem[0].problem_id);
        "
      />

      <!-- Functionality of Clone Button -->
      <q-btn
        color="primary"
        icon="o_file_copy"
        label="Copy"
        @click="
          copyProblem(parent_id, parent_name, selectedProblem[0].problem_id)
        "
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
      <q-btn
        color="green"
        icon="o_account_tree"
        label="Solve the Problem"
        padding="6px 190px "
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
    <!-- ---------------------------------------------------------------------- -->
    <!-- # Record Actions for Initial State  and Final State-->
    <!-- intial state -->
    <div
      class="q-pa-md q-gutter-sm column"
      style="width: 600px"
      v-if="displayActions == true"
    >
      <!-- <div
      class="q-pa-md q-gutter-sm"
      style="width: 600px"
      v-if="displayActions == true"
    > -->
      <!-- <DemoActions /> -->
      <!-- <div class="q-pa-md q-gutter-sm" style="width: 600px"> -->
      <div class="q-pa-md q-gutter-sm" style="width: 600px">
        <q-table
          title="Record Initial State"
          :columns="actioncolumns"
          :rows="cr_actions"
          row-key="action_id"
          selection="single"
          v-model:selected="selectedAction"
        >
          <!-- <template v-slot:top-right>
        <q-btn color="" disable label="{{selectedProblem[0].demo_id}}" @click="addRow" />
      </template> -->
          <!-- {{ selectedProblem[0].demo_id }} -->
        </q-table>
        <!-- <div>
      {{ cr_actions }}
      </div> -->
        <!-- functionality of ADD Button -->
        <!-- add/edit/delete--is(initialstate)actions prompts -->
        <q-btn
          icon="o_add"
          color="primary"
          label="Add"
          align="center"
          @click="addisactionprompt = true; state='init'"
        />

        <!-- Functionality of Edit Button -->
        <!-- <q-btn
          color="primary"
          icon="o_drive_file_rename_outline"
          label="Edit"
          @click="editisactionprompt = true"
        /> -->

        <!-- Functionality of Delete Button -->
        <q-btn
          color="red"
          icon="o_delete"
          label="Delete"
          align="center"
          @click="deleteisactionprompt = true"
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
            generateProbSequence(
              parent_id,
              parent_name,
              state='is',
              selectedProblem[0].problem_id
            );
            showSequence = true;
          "
        />
        <!-- Add Actions -->
        <q-dialog v-model="addisactionprompt" persistent auto-close="false">
          <q-card style="min-width: 1200px">
            <q-card-section>
              <div class="text-h6">Initial State Data</div>
            </q-card-section>

            <q-card-section class="q-pt-none">
              <q-form class="q-gutter-md">
                <!-- <q-input
                filled
                v-model="aname"
                type="text"
                label="Action Name *"
                hint="put block"
                lazy-rules
                :rules="[
                  (val) => (val && val.length > 0) || 'Please type something',
                ]"
              /> -->
              </q-form>
            </q-card-section>

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
                  addProbAction(
                    parent_id,
                    parent_name,
                    state,
                    selectedProblem[0].problem_id,
                    selectedProblem[0].problem_name,
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
        <q-dialog v-model="deleteisactionprompt" persistent auto-close="false">
          <q-card style="min-width: 350px">
            <q-card-section>
              <div class="text-h6">
                Do you want to delete action:
                {{ selectedAction[0].action_name }}
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
                  deleteProbAction(
                    parent_id,
                    parent_name,
                    state,
                    selectedProblem[0].problem_id,
                    selectedAction[0].action_id
                  )
                "
              />
            </q-card-actions>
          </q-card>
        </q-dialog>

        <!-- edit action -->
        <q-dialog v-model="editisactionprompt" persistent auto-close="false">
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
                  editProbAction(
                    parent_id,
                    parent_name,
                    state,
                    selectedProblem[0].problem_id,
                    selectedAction[0].action_id,
                    aname
                  );
                  initForm();
                "
              />
            </q-card-actions>
          </q-card>
        </q-dialog>

        <!-- </div> -->
      </div>
      <br />
      <br />
      <br />
      <br />
      <br />
      <!-- Final State -->
      <div class="q-pa-md q-gutter-sm" style="width: 600px">
        <q-table
          title="Record Final State"
          :columns="actioncolumns"
          :rows="cr_actions"
          row-key="action_id"
          selection="single"
          v-model:selected="selectedAction"
        >
          <!-- <template v-slot:top-right>
        <q-btn color="" disable label="{{selectedProblem[0].demo_id}}" @click="addRow" />
      </template> -->
          <!-- {{ selectedProblem[0].demo_id }} -->
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
          @click="addfsactionprompt = true"
        />
        <!-- Functionality of Edit Button -->
        <!-- <q-btn
          color="primary"
          icon="o_drive_file_rename_outline"
          label="Edit"
          @click="editfsactionprompt = true"
        /> -->

        <!-- Functionality of Delete Button -->
        <q-btn
          color="red"
          icon="o_delete"
          label="Delete"
          align="center"
          @click="deletefsactionprompt = true"
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
            generateProbSequence(
              parent_id,
              parent_name,
              state='fs',
              selectedProblem[0].demo_id
            );
            showSequence = true;
          "
        />
        <!-- Add Actions -->
        <q-dialog v-model="addfsactionprompt" persistent auto-close="false">
          <q-card style="min-width: 1200px">
            <q-card-section>
              <div class="text-h6">Final State Data</div>
            </q-card-section>

            <q-card-section class="q-pt-none">
              <q-form class="q-gutter-md">
                <!-- <q-input
                filled
                v-model="aname"
                type="text"
                label="Action Name *"
                hint="put block"
                lazy-rules
                :rules="[
                  (val) => (val && val.length > 0) || 'Please type something',
                ]"
              /> -->
              </q-form>
            </q-card-section>

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
                  addProbAction(
                    parent_id,
                    parent_name,
                    state,
                    selectedProblem[0].problem_id,
                    selectedProblem[0].problem_name,
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
        <q-dialog v-model="deletefsactionprompt" persistent auto-close="false">
          <q-card style="min-width: 350px">
            <q-card-section>
              <div class="text-h6">
                Do you want to delete this action:
                <!-- {{ selectedAction[0].action_id }} -->
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
                  deleteProbAction(
                    parent_id,
                    parent_name,
                    state,
                    selectedProblem[0].problem_id,
                    selectedAction[0].action_id
                  )
                "
              />
            </q-card-actions>
          </q-card>
        </q-dialog>

        <!-- edit action -->
        <q-dialog v-model="editfsactionprompt" persistent auto-close="false">
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
                  editProbAction(
                    parent_id,
                    parent_name,
                    state,
                    selectedProblem[0].problem_id,
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
      </div>
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
let displayActions = ref(false);
let gid_store = global_pid();
let parent_name = gid_store.getgname();
let parent_id = gid_store.getgid();
let p_name = ref("");

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
    align: "left",
    field: (row) => row.problem_name,
    format: (val) => `${val}`,
    sortable: true,
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
    .post("/addproblem", payload)
    .then((response) => {
      // problems.value = response.data.problems;
      getProblems(pt_id, pt_name);
      // books.value = response.data.books
      $q.notify({
        color: "green",
        position: "top",
        message: "Problem Created",
        icon: "o_done",
        timeout: 500,
      });
    })
    .catch(() => {
      $q.notify({
        color: "negative",
        position: "top",
        message: "Failed to create Problem",
        icon: "report_problem",
        timeout: 500,
      });
    });
}

function getProblems(pt_id, pt_name) {
  const payload = {
    parent_id: pt_id,
    parent_name: pt_name,
  };
  api
    .post("/getproblems", payload)
    .then((response) => {
      problems.value = response.data.problems;
      // books.value = response.data.books
      $q.notify({
        color: "green",
        position: "top",
        message: "Problems Retrieved",
        icon: "o_done",
        timeout: 500,
      });
    })
    .catch(() => {
      $q.notify({
        color: "negative",
        position: "top",
        message: "Problem Loading Failed: Server Unavailable",
        icon: "report_problem",
        timeout: 500,
      });
    });
}
// -------------########## For initial state
// -------------------- for actions section -----------------
let addisactionprompt = ref(false);
let deleteisactionprompt = ref(false);
let editisactionprompt = ref(false);
let addfsactionprompt = ref(false);
let deletefsactionprompt = ref(false);
let editfsactionprompt = ref(false);
let selectedAction = ref([]);
let state = ref('');
// let actions = ref([]);
let cr_actions = ref([]);
// const aname = ref("");
const aname = "action";
let astore = action_parameter();
let gpidstore = global_pid();
let ac_sequence = ref([]);
// let action_sequence = ref([])
let domain = ref("");
let domain_fomatted = ref("");

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

function addProbAction(pt_id, pt_name, state, problem_id, problem_name, aname, aparam) {
  const payload = {
    project_id: pt_id,
    project_name: pt_name,
    state: state,
    problem_id: problem_id,
    problem_name: problem_name,
    action_name: aname,
    action_parameters: aparam,
    // p_problems: 0,
    // p_actions: 0,
  };
  api
    .post("/addprobactions", payload)
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

function deleteProbAction(pt_id, pt_name, state, problem_id, act_id) {
  const payload = {
    parent_id: pt_id,
    parent_name: pt_name,
    state: state,
    problem_id: problem_id,
    action_id: act_id,
  };
  api
    .post("/deleteprobaction", payload)
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

function editProbAction(pt_id, pt_name, state, problem_id, act_id, act_name) {
  const payload = {
    parent_id: pt_id,
    parent_name: pt_name,
    state: state,
    problem_id: problem_id,
    action_id: act_id,
    action_name: act_name,
  };
  api
    .post("/updateprobaction", payload)
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

// function generateDomain(pt_id, pt_name) {
//   const payload = {
//     parent_id: pt_id,
//     parent_name: pt_name,
//   };
//   api
//     .post("/generatedomain", payload)
//     .then((response) => {
//       domain.value = response.data.ppdl_content;
//       gpidstore.setdomain(domain.value);
//       $q.notify({
//         color: "green",
//         position: "top",
//         message: "Domain Generated",
//         icon: "o_done",
//         timeout: 500,
//       });
//     })
//     .catch(() => {
//       $q.notify({
//         color: "negative",
//         position: "top",
//         message: "Failed to Generate Domain",
//         icon: "report_problem",
//         timeout: 500,
//       });
//     });
// }

function generateProbSequence(pt_id, pt_name, state, problem_id) {
  const payload = {
    parent_id: pt_id,
    parent_name: pt_name,
    state: state,
    problem_id: problem_id,
  };
  api
    .post("/generateprobsequence", payload)
    .then((response) => {
      getSequence(pt_id, pt_name, state, problem_id);
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

getProblems(parent_id, parent_name);
</script>

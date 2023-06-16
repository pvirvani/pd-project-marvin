<template>
  <div class="q-pa-md q-gutter-sm row">
    <div>
      <GeneralButtons></GeneralButtons>
    </div>
    <!-- parameter {{ lego_data }} -->
    <div class="q-pa-md q-gutter-sm" style="width: 800px">
      <q-table
        title="Actions"
        :columns="columns"
        :rows="actions"
        row-key="action_id"
        selection="single"
        v-model:selected="selectedAction"
      >
      </q-table>
      <!-- functionality of ADD Button -->
      <q-btn
        icon="o_file_open"
        color="primary"
        label="Add"
        @click="addprompt = true"
      />

      <!-- add prompt for flask api -->
      <q-dialog v-model="addprompt" persistent auto-close="false"> 
        <q-card style="min-width: 1200px">
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
                hint="put block"
                lazy-rules
                :rules="[
                  (val) => (val && val.length > 0) || 'Please type something',
                ]"
              />
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
                addAction(aname, astore.getparameter()[0]);
                initForm();
              "
              v-close-popup
            />
            <!-- @click="addProject(pname, pproblems, pactions)" -->
            <!-- <q-btn flat label="Reset" type="reset" @click="initForm()" /> -->
          </q-card-actions>
        </q-card>
      </q-dialog>

      <!-- </q-page> -->
      <div style="background-color: white">
        <p>
          <br />
          <br />
        </p>
      </div>
    </div>
  </div>
</template>

<script setup>
import { ref } from "vue";
import { useQuasar } from "quasar";
import { api } from "src/boot/axios";
import GeneralButtons from "src/components/GeneralButtons.vue";
import RosConnection from "src/pages/RosConnection.vue";
import { action_parameter } from "src/stores/action_parameter";
let addprompt = ref(false);
let selectedAction = ref([]);
let actions = ref([]);
let aname = ref("");

let xname = ref("");
const $q = useQuasar();
let astore = action_parameter();

const columns = [
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

function addAction(aname, aparam) {
  const payload = {
    action_name: aname,
    action_parameters: aparam,
    // p_problems: 0,
    // p_actions: 0,
  };
  api
    .post("/actions", payload)
    .then((response) => {
      getActions();
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
function initForm() {
  aname.value = "";
  aparam.value = "";
  // pactions.value = "";
}

getActions();
</script>

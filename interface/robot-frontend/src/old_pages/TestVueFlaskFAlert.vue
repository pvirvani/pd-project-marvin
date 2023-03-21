
<template>
    <div>
        <q-table title="Projects"
                 :columns="columns"
                 :rows="projects"
                 row-key="p_id"
                 selection="single"
                 v-model:selected="selectedProject">
        </q-table>
    </div>

    <div class="q-pa-md q-gutter-sm">

        <!-- functionality of ADD Button -->
        <q-btn icon="o_file_open"
               color="primary"
               label="Add"
               @click="addprompt = true" />

        <!-- functionality of Open Button -->
        <q-btn to="/demo"
               icon="o_file_open"
               color="primary"
               label="Open" />


        <!-- Functionality of Clone Button -->
        <q-btn color="primary"
               icon="o_copy"
               label="Copy"
               @click="copyProject(selectedProject[0])" />

        <!-- Functionality of Clone Button -->
        <!-- <q-btn color="primary"
               icon="o_drive_file_rename_outline"
               label="Edit"
               @click="editSelected(selected)" /> -->

        <q-btn color="primary"
               icon="o_drive_file_rename_outline"
               label="Edit"
               @click="editprompt = true" />


        <!-- Functionality of Delete Button -->
        <q-btn color="red"
               icon="o_delete"
               label="Delete"
               @click="deleteprompt = true"
               type="submit" />

        <!-- <div>
            selected id = {{ selectedProject[0] }}
        </div> -->

        <!-- Add Project Prompt -->
        <!-- <q-dialog v-model="addprompt"
                  persistent
                  auto-close="false">
            <q-card style="min-width: 350px">
                <q-card-section>
                    <div class="text-h6">Project Name</div>
                </q-card-section>

                <q-card-section class="q-pt-none">
                    <q-input dense
                             v-model="item"
                             type="text"
                             clearable
                             autofocus
                             @keyup.enter="addProject(pname)"
                             v-close-popup />
                </q-card-section>

                <q-card-actions align="right"
                                class="text-primary">
                    <q-btn flat
                           label="Cancel"
                           v-close-popup />
                    <q-btn flat
                           label="OK"
                           v-close-popup
                           type="submit"
                           @click="addProject(pname)" />
                </q-card-actions>
            </q-card>
        </q-dialog> -->
        <!-- add prompt for flask api -->
        <q-dialog v-model="addprompt"
                  persistent
                  auto-close="false">
            <q-card style="min-width: 350px">
                <q-card-section>
                    <div class="text-h6">Project Data</div>
                </q-card-section>

                <q-card-section class="q-pt-none">
                    <q-form class="q-gutter-md">
                        <q-input filled
                                 v-model="pname"
                                 type="text"
                                 label="Project Name *"
                                 hint="Book Title/Name"
                                 lazy-rules
                                 :rules="[val => val && val.length > 0 || 'Please type something']" />

                        <q-input filled
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
                                 :rules="[val => val && val.length > 0 || 'Please type something']" />
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

                <q-card-actions align="right"
                                class="text-primary">
                    <q-btn flat
                           label="Cancel"
                           v-close-popup />
                    <q-btn flat
                           label="OK"
                           v-close-popup
                           type="submit"
                           @click="addProject(pname, pproblems, pactions)" />
                    <q-btn flat
                           label="Reset"
                           type="reset"
                           @click="initForm()" />
                </q-card-actions>
            </q-card>
        </q-dialog>

        <!-- Edit Project Prompt -->
        <!-- <q-dialog v-model="editprompt"
                  persistent
                  auto-close="false">
            <q-card style="min-width: 350px">
                <q-card-section>
                    <div class="text-h6">Project Name</div>
                </q-card-section>

                <q-card-section class="q-pt-none">
                    <q-input dense
                             v-model="item_rename"
                             type="text"
                             clearable
                             autofocus
                             @keyup.enter="editSelected(selected, item_rename)"
                             v-close-popup />
                </q-card-section>

                <q-card-actions align="right"
                                class="text-primary">
                    <q-btn flat
                           label="Cancel"
                           v-close-popup />
                    <q-btn flat
                           label="Rename"
                           v-close-popup
                           type="submit"
                           @click="editSelected(selected, item_rename)" />
                </q-card-actions>
            </q-card>
        </q-dialog> -->
        <!-- edit prompt for flask api -->
        <q-dialog v-model="editprompt"
                  persistent
                  auto-close="false">
            <q-card style="min-width: 350px">
                <q-card-section>
                    <div class="text-h6">Project Data</div>
                </q-card-section>

                <q-card-section class="q-pt-none">
                    <q-form class="q-gutter-md">
                        <q-input filled
                                 v-model="pname"
                                 type="text"
                                 label="Project Name *"
                                 hint="Book Title/Name"
                                 lazy-rules
                                 :rules="[val => val && val.length > 0 || 'Please type something']" />

                        <q-input filled
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
                                 :rules="[val => val && val.length > 0 || 'Please type something']" />
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

                <q-card-actions align="right"
                                class="text-primary">
                    <q-btn flat
                           label="Cancel"
                           v-close-popup />
                    <q-btn flat
                           label="Update"
                           v-close-popup
                           type="submit"
                           @click="editProject(pname, pproblems, pactions, selectedProject[0].p_id)" />
                    <q-btn flat
                           label="Reset"
                           type="reset"
                           @click="initForm()" />
                </q-card-actions>
            </q-card>
        </q-dialog>
        <!-- delete prompt flask api -->
        <!-- edit prompt for flask api -->
        <q-dialog v-model="deleteprompt"
                  persistent
                  auto-close="false">
            <q-card style="min-width: 350px">
                <q-card-section>
                    <div class="text-h6">Do you want to delete: {{ selectedProject[0].p_name }}
                    </div>
                    <!-- <div> {{}}</div> -->
                </q-card-section>
                <q-card-actions align="right"
                                class="text-primary"
                                label="Do you want to delete the Project">
                    <q-btn flat
                           label="Cancel"
                           v-close-popup />
                    <q-btn flat
                           label="Delete"
                           v-close-popup
                           type="submit"
                           @click="deleteProject(selectedProject[0].p_id)" />
                </q-card-actions>
            </q-card>
        </q-dialog>
    </div>

</template>

<script setup>

import { ref } from 'vue'
import { api } from '../boot/axios'
import { useQuasar } from 'quasar'

let addprompt = ref(false);
let editprompt = ref(false);
let deleteprompt = ref(false);

const $q = useQuasar();
const projects = ref([]);
// const pname = ref(false)
let pname = ref("")
let pproblems = ref("")
let pactions = ref("")
let pid = ref([])
let selectedProject = ref([]);
// const bookform = ref('');
// const read = ref([])


// data.value = "pardeep";

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
        name: 'p_name',
        label: 'Project Name',
        required: true,
        align: 'center',
        field: row => row.p_name,
        format: val => `${val}`,
        sortable: true
    },
    {
        name: 'p_problems',
        label: 'Problems',
        required: true,
        align: 'center',
        field: row => row.p_problems,
        format: val => `${val}`,
        sortable: true
    },
    {
        name: 'p_actions',
        label: 'Actions',
        required: true,
        align: 'center',
        field: row => row.p_actions,
        format: val => `${val}`,
        sortable: true
    },
];


function getProject() {
    api.get('/projects')
        .then((response) => {
            projects.value = response.data.projects
        })
        .catch(() => {
            $q.notify({
                color: 'negative',
                position: 'top',
                message: 'Loading failed',
                icon: 'report_problem'
            })
        })
};
function addProject(pname, pproblems, pactions) {
    const payload = {
        p_name: pname,
        p_problems: pproblems,
        p_actions: pactions
    }
    api.post('/projects', payload)
        .then((response) => {
            getProject()
            // books.value = response.data.books
            $q.notify({
                color: 'green',
                position: 'top',
                message: 'Project Created',
                icon: 'o_done',
                timeout: 500
            })

        })
        .catch(() => {
            $q.notify({
                color: 'negative',
                position: 'top',
                message: 'Loading failed',
                icon: 'report_problem',
                timeout: 500
            })
        })
};

function editProject(pname, pproblems, pactions, pid) {
    const payload = {
        p_id: pid,
        p_name: pname,
        p_problems: pproblems,
        p_actions: pactions
    }
    api.post('/updateproject', payload)
        .then((response) => {
            getProject()
            // books.value = response.data.books
            $q.notify({
                color: 'green',
                position: 'top',
                message: 'Project Updated',
                icon: 'o_done',
                timeout: 500
            })
            initForm()
        })
        .catch(() => {
            $q.notify({
                color: 'negative',
                position: 'top',
                message: 'Loading failed',
                icon: 'report_problem',
                timeout: 500
            })
        })
};
function copyProject(selecteddata) {
    const payload = {
        p_id: selecteddata.p_id,
        p_name: selecteddata.p_name,
        p_problems: selecteddata.p_problems,
        p_actions: selecteddata.p_actions
    }
    api.post('/copyproject', payload)
        .then((response) => {
            getProject()
            // books.value = response.data.books
            $q.notify({
                color: 'green',
                position: 'top',
                message: 'Project Copied',
                icon: 'o_done',
                timeout: 500
            })
            // initForm()
        })
        .catch(() => {
            $q.notify({
                color: 'negative',
                position: 'top',
                message: 'Loading failed',
                icon: 'report_problem',
                timeout: 500
            })
        })
};
function deleteProject(pid) {
    const payload = {
        p_id: pid
    }
    api.post('/deleteproject', payload)
        .then((response) => {
            getProject()
            // books.value = response.data.books
            $q.notify({
                color: 'red',
                position: 'top',
                message: 'Project Deleted',
                icon: 'o_done',
                timeout: 500
            })
            // initForm()
        })
        .catch(() => {
            $q.notify({
                color: 'negative',
                position: 'top',
                message: 'Loading failed',
                icon: 'report_problem',
                timeout: 500
            })
        })
};
function initForm() {
    pname.value = '';
    pproblems.value = '';
    pactions.value = '';
};

getProject();

</script>
<style>
#projecttable {
    font-family: Arial, Helvetica, sans-serif;
    border-collapse: collapse;
    width: 100%;
}

#projecttable td,
#projecttable th {
    border: 1px solid #ddd;
    padding: 8px;
}

#projecttable tr:nth-child(even) {
    background-color: #f2f2f2;
}

#projecttable tr:hover {
    background-color: #ddd;
}

#projecttable th {
    padding-top: 12px;
    padding-bottom: 12px;
    text-align: center;
    background-color: #04AA6D;
    color: white;
}
</style>
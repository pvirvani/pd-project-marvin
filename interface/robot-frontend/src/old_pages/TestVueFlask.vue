
<template>
    <div>
        <q-table title="Projects"
                 :columns="columns"
                 :rows="projects"
                 row-key="name"
                 selection="single"
                 v-model:selected="selected">
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
               label="Copy" />

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
               @click="deleteSelected(selected)"
               type="submit" />

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
        <q-dialog v-model="addprompt"
                  persistent
                  auto-close="false">
            <q-card style="min-width: 350px">
                <q-card-section>
                    <div class="text-h6">Book Data</div>
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
        <q-dialog v-model="editprompt"
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
        </q-dialog>
    </div>

</template>

<!-- <script setup>

import { useCounterStore } from 'stores/example-store';
import { projectStore } from 'stores/project-table';
import { useQuasar } from 'quasar';
import { ref } from 'vue';
import { storeToRefs } from 'pinia';


const $q = useQuasar();

// let store = useCounterStore();
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
        name: 'pname',
        label: 'ProjectName',
        required: true,
        align: 'center',
        field: row => row.pname,
        format: val => `${val}`,
        sortable: true
    },
    {
        name: 'projectProblems',
        label: 'AvailableProblems',
        required: true,
        align: 'center',
        field: row => row.projectProblems,
        format: val => `${val}`,
        sortable: true
    },
    {
        name: 'projectActions',
        label: 'AvailableActions',
        required: true,
        align: 'center',
        field: row => row.projectActions,
        format: val => `${val}`,
        sortable: true
    },
];


const projName = ref('');
const proj_store = projectStore();
const { projectId } = storeToRefs(proj_store);10
let editprompt = ref(false);
let addprompt = ref(false);
let item_rename = ref(null); // for renaming project
let item = ref(null); // for adding project

// let rows = proj_store.projectName;

function addProjectandDelete(item) {
    if (item.length == 0) {
        pass
    }
    // invokes function in the store:
    else {
        proj_store.addProject(item)
        item.value = ref(null)
        item.value.resetValidation()
    }

};

let selected = ref([]);


function deleteSelected(selected) {
    proj_store.deleteProject(selected[0].id);



};
function editSelected(selected, item_rename) {
    if (item_rename == null || item_rename == "") {
        pass;
    } else {
        proj_store.editProject(selected[0].id, item_rename);
        return item_rename.value = ''
    }


};



// let p_i = 0;

</script> -->

<script setup>

import { ref } from 'vue'
import { api } from '../boot/axios'
import { useQuasar } from 'quasar'
let addprompt = ref(false);
let editprompt = ref(false);

const $q = useQuasar();
const projects = ref([]);
// const pname = ref(false)
let pname = ref("")
let pproblems = ref("")
let pactions = ref("")
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

<template>

    <div class="q-pa-md">
        <div class="q-gutter-md row items-start">

            <q-form class="row"
                    @submit.prevent="addProjectandDelete(projName)">
                <q-input v-model="projName"
                         type="text"
                         hint="Project Name"
                         outlined />
                &emsp;

                <q-btn icon="o_add"
                       color="primary"
                       style="max-height:55px"
                       type="submit"> Add </q-btn>
            </q-form>
        </div>
    </div>

    <br />

    <div>
        <q-table title="Projects"
                 :columns="columns"
                 :rows="proj_store.projectName"
                 row-key="id"
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
        <q-dialog v-model="addprompt"
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
                             @keyup.enter="addProjectandDelete(item)"
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
                           @click="addProjectandDelete(item)" />
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

<script setup>

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
const { projectId } = storeToRefs(proj_store);
let editprompt = ref(false);
let addprompt = ref(false);
let item_rename = ref(null); // for renaming project
let item = ref(null); // for adding project

// let rows = proj_store.projectName;

async function addProjectandDelete(item) {
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
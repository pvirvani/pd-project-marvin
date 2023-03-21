
<template>
    <!-- Counter: {{ store.counter }}
    <q-btn @click="addN">
        Count is: {{ store.counter }}
    </q-btn> -->
    <div class="q-pa-md">
        <div class="q-gutter-md row items-start">

            <q-form class="row"
                    @submit.prevent="addProjectandDelete(projName)">
                <q-input v-model="projName"
                         type="text"
                         hint="Project Name"
                         outlined />
                &emsp;
                <!-- <q-input v-model="projName"
                         type="text"
                         outlined />

                &emsp; -->
                <q-btn icon="o_add"
                       color="primary"
                       style="max-height:55px"
                       type="submit"> Add </q-btn>
            </q-form>
        </div>
    </div>
    <div class="flex flex-center">
        <table class="flex-center"
               id="projecttable">
            <!-- <tr>
                <th>
                    ID
                </th>
                <th>
                    Name
                </th>
                <th>
                    Available Problems
                </th>
                <th>
                    Available Actions
                </th>
            </tr> -->
            <tr v-for="(name, id) in proj_store.projectName"
                :key="id">
                <!-- <td>
                    {{ name.id + 1 }}
                </td>
                <td>
                    {{ name.pname }}
                </td>
                <td>
                    {{ name.projectProblems }}
                </td>
                <td>
                    {{ name.projectActions }}
                </td> -->
            </tr>
        </table>
    </div>
    <br />
    <!-- <div>
        <q-btn icon="o_add"
               color="primary"
               label="Add"
               @click="newProject">

        </q-btn>
    </div> -->

    <div>
        <q-table title="Projects"
                 :columns="columns"
                 :rows="proj_store.projectName"
                 row-key="id"
                 selection="single"
                 v-model:selected="selected">
        </q-table>
    </div>

    <div>
        <!-- items store : {{ proj_store.projectName }} -->
        <!-- <br /> -->
        <!-- total-items-store : {{ proj_store.projectName.length }} -->

        <!-- <br /> -->
        <!-- rows : {{ rows[0] }} -->
        <!-- <br /> -->
        <!-- rows-items : {{ rows[0].length }} -->
        <!-- {{ rows[0] }} -->
        <!-- Selected: {{ JSON.stringify(selected) }} -->
        <!-- Selected-ID: {{ JSON.stringify(selected) }} -->
        <!-- <br /> -->
        <!-- selected: {{ selected }} -->
        <!-- <br /> -->

        <!-- sel_pro : {{ proj_store.projectName[selected[0].id] }} -->
        <!-- <br /> -->
        <!-- selected-id : {{ selected[0].id }} -->
        <!-- id-in-rows : {{ rows[0].indexOf(selected[0].id) }} -->
        <!-- id-in-rows : {{ rows[0].map(function (o) { return o.id; }).indexOf(selected[0].id) }} -->
        <!-- selected: {{ selected.length }} -->
        <!-- <br /> -->
        <!-- t items : {{ proj_store.projectName.length }} -->
        <!-- <br /> -->
        <!-- indexes : {{ indx }} -->
        <!-- <br /> -->
        <!-- s_id : {{ s_id }} -->
        <!-- <br /> -->
        <!-- value : {{ vl }} -->
        <!-- <br /> -->
        <!-- rows : {{ rows[0][2] }} -->
    </div>

    <div class="q-pa-md q-gutter-sm">

        <!-- functionality of ADD Button -->
        <!-- <q-btn color="primary"
               icon="o_add"
               label="ADD"
               @click="prompt_add" /> -->

        <!-- Functionality of Open Button -->
        <!-- <q-btn to="/openedproject" icon="o_file_open" color="primary" label="Open" /> -->
        <q-btn to="/demo"
               icon="o_file_open"
               color="primary"
               label="Open" />


        <!-- Functionality of Clone Button -->
        <q-btn color="primary"
               icon="o_copy"
               label="Copy" />

        <!-- Functionality of Clone Button -->
        <q-btn color="primary"
               icon="o_drive_file_rename_outline"
               label="Rename"
               @click="showid(selected)" />

        <!-- Functionality of Delete Button -->
        <q-btn color="red"
               icon="o_delete"
               label="Delete"
               @click="deleteSelected(selected)"
               type="submit" />
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

// let rows = proj_store.projectName;
function addProjectandDelete(item) {
    if (item.length === 0) {
        return
    }
    // invokes function in the store:
    proj_store.addProject(item)
    // rows.push(proj_store.projectName)
    projName.value = ''
};
// let selected = ref([]);
let selected = ref([]);
let indx = ref([]);
let s_id = ref([]);
let vl = ref([]);

function deleteSelected(selected) {
    if (indx.value.length >= 1) {
        indx.value.length = 0;
        indx.value.push(selected[0].id)
    }
    else {
        indx.value.push(selected[0].id)
    }

    // rows[0].splice(rows[0].map(function (o) { return o.id; }).indexOf(selected[0].id), 1)
    proj_store.deleteProject(selected[0].id);



};
function showid(selected) {
    s_id.value.push(selected[0].id)

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
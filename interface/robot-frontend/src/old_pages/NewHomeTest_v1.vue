<template>
    <div class="q-pa-md">
        <q-table title="Available Projects"
                 :columns="columns"
                 row-key="name"
                 :pagination="pagination"
                 :data="table_data">
            <template v-slot:body="props">
                <q-tr v-for="(tdata, i) in table_data"
                      :key="i"
                      :props="props">
                    <q-td key="name"
                          :props="props">{{ props.row.names[index] }}</q-td>
                    <q-td :key="actions"
                          :props="props">{{ props.row.actions[index] }}</q-td>
                    <q-td :key="problems"
                          :props="props">{{ props.row.problems[index] }}</q-td>

                </q-tr>
            </template>

        </q-table>
        <!-- <table>
            <tr>
                <th> Name </th>const table_data = {
            names: [],
            problems: [],
            actions: [],


        };
                <th> Actions </th>
                <th> Problems </th>
            </tr>
            <tr v-for="tdata in table_data"
                :key="tdata.id">
                <td> {{ tdata.names }} </td>
                <td> {{ tdata.problems }}</td>
                <td> {{ tdata.actions }} </td>
            </tr>
        </table> -->

    </div>
    <div class="q-pa-md q-gutter-sm">

        <!-- functionality of ADD Button -->
        <q-btn color="primary"
               icon="o_add"
               label="ADD"
               @click="prompt_add" />

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
               @click="prompt_rename" />

        <!-- Functionality of Delete Button -->
        <q-btn color="red"
               icon="o_delete"
               label="Delete"
               @click="confirm_delete" />
    </div>
    <!-- <div>
        <p v-for="tdata in table_data"
           :key="tdata.tdata">
            {{ tdata }} created</p>
    </div> -->
    <!-- <p> {{ table_data }} created</p> -->
</template>

<script>
import { defineComponent, ref } from 'vue';
import { useQuasar } from 'quasar'
import OnProblemValidation from './OnProblemValidation.vue';

export default {
    defineComponent() {
        name: 'NewHomeTest';
    }, // define component end
    setup() {
        const $q = useQuasar();
        const inname = "Pardeep";
        const columns = [
            // {
            //     name: 'p_id',
            //     label: 'Index',
            // },
            {
                name: 'names',
                label: 'Name',
                required: 'true',
            },
            {
                name: 'actions',
                label: 'Actions',
            },
            {
                name: 'problems',
                label: 'Problems',

            },
            // {
            //     name: 'p_date',
            //     label: 'Date',
            // },
        ]
        const table_data = {
            names: [],
            problems: [],
            actions: [],


        };

        return {
            prompt_add() {
                $q.dialog({
                    title: 'Enter Project Name:',
                    // message: 'Enter Project Name:',
                    prompt: {
                        model: '',
                        isValid: val => val.length > 2,
                        type: 'text'
                    },
                    cancel: true,
                    persistent: true
                }).onOk(data => {
                    table_data.names.push(data);
                    table_data.problems.push(6);
                    table_data.actions.push(3);
                    // console.log(data + ' Created');
                    // console.log(table_data.names + ' created');
                    console.log(table_data.names + ' has been created with ' + table_data.actions + ' actions and ' + table_data.problems + ' problems');
                })
            },
            inname,
            columns,
            table_data,
        }// retun end

    }//setup end
}// export end

</script>


<style>
table {
    font-family: arial, sans-serif;
    border-collapse: collapse;
    width: 100%;
}

td,
th {
    border: 1px solid #dddddd;
    text-align: left;
    padding: 8px;
}

tr:nth-child(even) {
    background-color: #dddddd;
}
</style>
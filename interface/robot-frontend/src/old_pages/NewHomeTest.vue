<template>
    <div class="q-pa-md">
        <q-table title="Available Projects"
                 :columns="columns">
        </q-table>
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
           :key="tdata">
            {{ tdata }} created</p>
    </div> -->
</template>

<script>
import { defineComponent, ref } from 'vue';
import { useQuasar } from 'quasar'

export default {
    defineComponent() {
        name: 'NewHomeTest';
    }, // define component end
    setup() {
        const $q = useQuasar();
        const inname = "Pardeep";
        const columns = [
            {
                name: 'p_id',
                label: 'Index',
            },
            {
                name: 'p_name',
                label: 'Name',
                required: 'true',
            },
            {
                name: 'p_actions',
                label: 'Actions',
            },
            {
                name: 'p_problems',
                label: 'Problems',

            },
            {
                name: 'p_date',
                label: 'Date',
            },
        ]
        const table_data = [];

        return {
            inname,
            columns,
            table_data,

            prompt_add() {
                $q.dialog({
                    title: 'Prompt_Add',
                    message: 'Enter Project Name:',
                    prompt: {
                        model: '',
                        isValid: val => val.length > 2,
                        type: 'text'
                    },
                    cancel: true,
                    persistent: true
                }).onOk(data => {
                    table_data.push(data);
                    console.log(data + ' Created')
                })
            },

        }// retun end

    }//setup end
}// export end

</script>
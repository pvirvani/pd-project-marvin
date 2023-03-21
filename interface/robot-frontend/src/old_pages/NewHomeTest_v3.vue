
<template>
    <div>
        NewHomeTest_v3
    </div>

    <q-virtual-scroll style="max-height: 300px;"
                      :items="heavyList"
                      separator
                      v-slot="{ item, index }">

        <q-item :key="index"
                dense>
            <q-item-section>
                <q-item-label>
                    #{{ index }} - {{ item.label }}
                </q-item-label>
            </q-item-section>

        </q-item>
    </q-virtual-scroll>


</template>


<script>
import { defineComponent } from 'vue'
const maxSize = 10
const heavyList = []

for (let i = 0; i < maxSize; i++) {
    heavyList.push({
        label: 'Option ' + (i + 1)
    })
}

export default {
    defineComponent() {
        name: 'NewHomeTest_v3'
    }, // define component end
    setup() {
        return {
            heavyList,

            showcontent() {
            },


        }//return end
    }//setup end
}// export end
</script>

<!-- <script>
import { defineComponent, ref } from 'vue';
import { useQuasar } from 'quasar'
import OnProblemValidation from './OnProblemValidation.vue';

export default {
    defineComponent() {
        name: 'NewHomeTest_v3';
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

</script> -->


<!-- <style>
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
</style> -->
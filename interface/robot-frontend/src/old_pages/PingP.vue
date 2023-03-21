<template>
    <div>
        <p>{{ data }}</p>
    </div>
</template>

<!-- <script setup>
const data = "Hello!";
</script> -->

<script setup>
import { ref } from 'vue'
import { api } from '../boot/axios'
import { useQuasar } from 'quasar'

const $q = useQuasar();
const data = ref(null);

// data.value = "pardeep";

function getMessage() {
    api.get('/ping')
        .then((response) => {
            data.value = response.data
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

getMessage();

// import axios from 'axios';
// const msg = "Hello!";
// class getMessage {
//     constructor() {
//         const path = 'http://localhost:5000/ping';
//         axios.get(path)
//             .then((res) => {
//                 this.msg = res.data;

//             })
//             .catch((error) => {
//                 console.error(error);
//             });
//     }
// };

</script>


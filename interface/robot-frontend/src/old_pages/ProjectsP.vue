<template>
    <div>

        pname = {{ projects.p_name }} == {{ pname }}
        <br />
        pproblem = {{ projects.p_name }} == {{ pproblems }}
        <br />
        pactions = {{ projects.p_name }} == {{ pactions }}
        <br />
        payload = {{ payload }}
    </div>
    <div class="q-pa-md q-gutter-sm">
        <div class="row">
            <div class="col-sm-10">
                <h1>Proejcts</h1>
                <hr><br><br>
                <q-btn icon="o_add"
                       color="primary"
                       label="Add Book"
                       @click="addprompt = true" />
                <br><br>
                <table class="table table-hover">
                    <thead>
                        <tr>
                            <th scope="col">Project Name</th>
                            <th scope="col">Problems</th>
                            <th scope="col">Actions</th>
                            <th></th>
                        </tr>
                    </thead>
                    <tbody>
                        <tr v-for="(project, index) in projects"
                            :key="index">
                            <td>{{ project.p_name }}</td>
                            <td>{{ project.p_problems }}</td>
                            <td>{{ project.p_actions }}</td>
                            <!-- <td>
                                <span v-if="book.read">Yes</span>
                                <span v-else>No</span>
                            </td> -->
                            <td>
                                <div class="btn-group"
                                     role="group">
                                    <button type="button"
                                            class="btn btn-warning btn-sm">Update</button>
                                    <button type="button"
                                            class="btn btn-danger btn-sm">Delete</button>
                                </div>
                            </td>
                        </tr>
                    </tbody>
                </table>
            </div>
        </div>
        <!-- Add Project Prompt -->
        <q-dialog v-model="addprompt"
                  persistent
                  auto-close="false">
            <q-card style="min-width: 350px">
                <q-card-section>
                    <div class="text-h6">Project Data</div>
                </q-card-section>

                <q-card-section class="q-pt-none">
                    <q-form @submit="onSubmit"
                            @reset="onReset"
                            class="q-gutter-md">
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
                                 label="Problems"
                                 lazy-rules
                                 :rules="[val => val && val.length > 0 || 'Please type something']" />
                        <q-input filled
                                 type="text"
                                 v-model="pactions"
                                 label="Actions *"
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
                           @click="addBook(pname, pproblems, pactions)" />
                    <q-btn flat
                           label="Reset"
                           v-close-popup
                           type="reset"
                           @click="onReset" />
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

const $q = useQuasar();
const projects = ref([]);
// const bread = ref(false)
const pname = ref("")
const pproblems = ref("")
const pactions = ref("")
// const bookform = ref('');
// const read = ref([])


// data.value = "pardeep";

function getBooks() {
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
function addBook(pname, pproblems, pactions) {
    const payload = {
        p_name: pname,
        p_problems: pproblems,
        p_actions: pactions
    }
    api.post('/projects', payload)
        .then((response) => {
            getBooks()
            // books.value = response.data.books
            initForm()
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
    this.btitle = '';
    this.bauthor = '';
    this.bread = false;
};
getBooks();

</script>
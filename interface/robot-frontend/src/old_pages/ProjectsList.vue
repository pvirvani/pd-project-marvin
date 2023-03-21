<template>
    <q-page class="flex flex-center">
        <div class="q-pa-md flex flex-center"
             style="max-width: 350px">
            <q-toolbar class="bg-primary text-white shadow-2">
                <q-toolbar-title class="flex flex-center">Available Projects</q-toolbar-title>
            </q-toolbar>

            <q-list bordered>
                <q-item v-for="project in projects"
                        :key="project.id"
                        class="q-my-sm"
                        clickable
                        v-ripple>
                    <q-item-section avatar>
                        <q-avatar color="primary"
                                  text-color="white">
                            {{ project.letter }}
                        </q-avatar>
                    </q-item-section>

                    <q-item-section>
                        <q-item-label>{{ project.name }}</q-item-label>
                        <q-item-label caption
                                      lines="1">{{ project.author }}</q-item-label>
                    </q-item-section>



                    <q-item-section side>
                        <q-icon name="calendar_today"
                                color="green" />
                    </q-item-section>
                </q-item>

                <q-separator />
                <!-- <q-item-label header>Offline</q-item-label>

                <q-item v-for="project in offline" :key="project.id" class="q-mb-sm" clickable v-ripple>
                    <q-item-section avatar>
                        <q-avatar>
                            <img :src="`https://cdn.quasar.dev/img/${project.avatar}`">
                        </q-avatar>
                    </q-item-section>

                    <q-item-section>
                        <q-item-label>{{ project.name }}</q-item-label>
                        <q-item-label caption lines="1">{{ project.email }}</q-item-label>
                    </q-item-section>

                    <q-item-section side>
                        <q-icon name="chat_bubble" color="grey" />
                    </q-item-section>
                </q-item> -->
            </q-list>
        </div>
    </q-page>
</template>
  
<script>
import { defineComponent } from 'vue'

const projects = [{
    id: 1,
    name: 'Project1',
    author: 'Mr. Martin',
    email: 'rjedrzej0@discuz.net',
    letter: 'M'
},
{
    id: 2,
    name: 'Project_2',
    author: 'Mr. Dalton',
    email: 'malessandrini1@marketwatch.com',
    letter: 'D'
},
    // , {
    //     id: 3,
    //     name: 'Elisabetta Wicklen',
    //     email: 'ewicklen2@microsoft.com',
    //     letter: 'E'
    // }, {
    //     id: 4,
    //     name: 'Seka Fawdrey',
    //     email: 'sfawdrey3@wired.com',
    //     letter: 'S'
    // }
]

// const offline = [{
//     id: 5,
//     name: 'Brunhilde Panswick',
//     email: 'bpanswick4@csmonitor.com',
//     avatar: 'avatar2.jpg'
// }, {
//     id: 6,
//     name: 'Winfield Stapforth',
//     email: 'wstapforth5@pcworld.com',
//     avatar: 'avatar6.jpg'
// }]

export default {

    setup() {
        return {
            projects
            // offline
        }
    },
    defineComponent() {
        name: 'ProjectsList'
    }
}
</script>
  
    
<!-- <script>
import { defineComponent } from 'vue'

// export default defineComponent({
//   name: 'IndexPage'
// })

export default {
    defineComponent() {
        name: 'OpenProject'
    }
}
</script>
     -->
<template>
  <q-page class="flex flex-center">
    <!-- <img
        alt="Quasar logo"
        src="~assets/quasar-logo-vertical.svg"
        style="width: 200px; height: 200px"
      > -->
    <!-- <div class="flex flex-center"> -->
    <!-- <div class="q-gutter-md" style="max-width: 300px"> -->
    <!-- <q-field label="Standard" stack-label>
          <template v-slot:control>
            <div class="self-center full-width no-outline" tabindex="0">Field content</div>
          </template>
        </q-field> -->

    <!-- <q-field filled label="Filled" stack-label>
          <template v-slot:control>
            <div class="self-center full-width no-outline" tabindex="0">Field content</div>
          </template>
        </q-field> -->

    <!--<q-field outlined label="Outlined" stack-label>
          <template v-slot:control>
            <div class="self-center full-width no-outline" tabindex="0">Field content</div>
          </template>
        </q-field> -->

    <!-- <q-field standout label="Standout" stack-label>
          <template v-slot:control>
            <div class="self-center full-width no-outline" tabindex="0">Field content</div>
          </template>
        </q-field> -->

    <!-- <q-field standout="bg-teal text-white" label="Custom standout" stack-label>
          <template v-slot:control>
            <div class="self-center full-width no-outline" tabindex="0">Field content</div>
          </template>
        </q-field> -->

    <!-- <q-field borderless label="Borderless" stack-label>
          <template v-slot:control>
            <div class="self-center full-width no-outline" tabindex="0">Field content</div>
          </template>
        </q-field> -->

    <!-- <q-field rounded filled label="Rounded filled" stack-label>
          <template v-slot:control>
            <div class="self-center full-width no-outline" tabindex="0">Field content</div>
          </template>
        </q-field> -->

    <!-- <q-field rounded outlined label="Enter Project Name" stack-label>
          <template v-slot:control>
            <div class="self-center full-width no-outline" tabindex="0">ProjectName</div>
          </template>
        </q-field> -->

    <!-- <q-field rounded standout label="Rounded standout" stack-label>
          <template v-slot:control>
            <div class="self-center full-width no-outline" tabindex="0">Field content</div>
          </template>
        </q-field> -->

    <!-- <q-field square filled label="Square filled" stack-label>
          <template v-slot:control>
            <div class="self-center full-width no-outline" tabindex="0">Field content</div>
          </template>
        </q-field> -->

    <!-- <q-field square outlined label="Square outlined" stack-label>
          <template v-slot:control>
            <div class="self-center full-width no-outline" tabindex="0">Field content</div>
          </template>
        </q-field> -->

    <!-- <q-field square standout label="Square standout" stack-label>
          <template v-slot:control>
            <div class="self-center full-width no-outline" tabindex="0">Field content</div>
          </template>
        </q-field> -->
    <!-- </div> -->
    <!-- </div> -->
    <div class="q-pa-md"
         style="max-width: 500px">

      <q-form @submit="onSubmit"
              @reset="onReset"
              class="q-gutter-md">
        <q-input rounded
                 outlined
                 v-model="name"
                 label="Type Project Name*"
                 hint="Project1 or Project-1"
                 lazy-rules
                 :rules="[ val => val && val.length > 0 || 'Please type project name']" />

        <!-- <q-input filled type="number" v-model="age" label="Your age *" lazy-rules :rules="[
          val => val !== null && val !== '' || 'Please type your age',
          val => val > 0 && val < 100 || 'Please type a real age'
        ]" /> -->

        <!-- <q-toggle v-model="accept" label="I accept the license and terms" /> -->

        <div class="q-pa-md flex flex-center">
          <q-btn rounded
                 label="Create"
                 type="submit"
                 color="primary" />
          <!-- <q-btn label="Reset" type="reset" color="primary" flat class="q-ml-sm" /> -->
        </div>
      </q-form>

    </div>
  </q-page>
</template>

<script>
import { defineComponent } from 'vue'

// export default defineComponent({
//   name: 'IndexPage'
// })

export default {
  defineComponent() {
    name: 'CreateNewProject'
  }
}
</script>

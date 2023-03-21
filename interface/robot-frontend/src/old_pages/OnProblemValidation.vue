<template>
  <!-- table starts from here -->

  <div style="background-color:rosybrown"
       class="q-pa-md">
    <div>
      <q-btn flat
             disable
             label=" Problem 1"></q-btn>
      <q-btn icon="verified"
             color="green"
             label="Validated !"></q-btn>
      &emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;
      <q-btn icon="o_"
             label="Execute&emsp;&emsp;"
             glossy
             color="green"></q-btn>

    </div>
    <div class="q-pa-md row items-start q-gutter-md">
      <q-card class="my-card"
              style="max-width:300px">
        <q-card-section class="bg-secondary text-white">
          <div class="text-h6">&emsp;&emsp;&emsp;&emsp;&emsp;Actions</div>
        </q-card-section>

        <q-card-section>
          Action 1 ()
        </q-card-section>

        <q-separator />
        <q-card-section>
          Action 1 ()
        </q-card-section>

        <q-separator />
        <q-card-section>
          Action 1 ()
        </q-card-section>

        <q-separator />
        <q-card-section>
          Action 1 ()
        </q-card-section>

        <q-separator />
        <q-card-section>
          Action 1 ()
        </q-card-section>

        <q-separator />

        <!-- <q-card-actions>
          <q-btn flat>Action 1</q-btn>
          <q-btn flat>Action 2</q-btn>
        </q-card-actions> -->
      </q-card>
      &emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;
      <q-card class="my-card"
              style="max-width:600px">
        <q-card-section class="bg-secondary text-white">
          <div class="text-h6">
            &emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;Visual
          </div>
        </q-card-section>

        <q-card-section>
          <q-img src="../assets/images/state_example.png" />
        </q-card-section>

        <q-separator />
        <q-card-section>
          Status:&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;Time:
          <p style="color:red">Stopped &emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;00:00</p>
        </q-card-section>

        <!-- <q-card-actions>
          <q-btn flat>Action 1</q-btn>
          <q-btn flat>Action 2</q-btn>
        </q-card-actions> -->
      </q-card>
    </div>
  </div>
  <!-- </q-page> -->
  <!-- <div style=" background-color:white">
            <p>
              <br>
              <br>
            </p>
    </div> -->


  <!-- <div style="background-color:wheat" class="q-pa-md">
    <div class="q-pa-md row items-start q-gutter-md flex flex-center">

      <q-card class="my-card">
        <q-card-section class="bg-grey-8 text-white">
          <div class="text-h6" style="text-align:center">Initial State</div>
        </q-card-section>
        <q-separator />
        <q-card-section class="bg-grey-6 text-white">
          <div class="text-h8" style="text-align:center; font-weight:bold">Visual</div>
        </q-card-section>
        <div>
          <br />
          <br />


          <p style="font-weight:bold; text-align:center"> Nothing to show</p>
          <br />
          <br />

        </div>
        <q-card-section class="bg-grey-6 text-white">
          <div class="text-h8" style="text-align:center; font-weight:bold">Preconditions</div>
        </q-card-section>
        <div>
          <br />
          <br />

          <p style="font-weight:bold; text-align:center"> No precondition available</p>
          <br />
          <br />


        </div>

        <q-card-actions vertical>
          <q-btn icon="o_photo_camera" color="blue-grey" glossy label="Capture">
            <q-tooltip class="bg-accent">capture initial
              state</q-tooltip>
          </q-btn>
        </q-card-actions>
      </q-card>

      <q-card class="my-card">
        <q-card-section class="bg-grey-8 text-white">
          <div class="text-h6" style="text-align:center">Goal State</div>
        </q-card-section>
        <q-separator />
        <q-card-section class="bg-grey-6 text-white">
          <div class="text-h8" style="text-align:center; font-weight:bold">Visual</div>
        </q-card-section>
        <div>
          <br />
          <br />
          <p style="font-weight:bold; text-align:center"> Nothing to show</p>
          <br />
          <br />

        </div>
        <q-card-section class="bg-grey-6 text-white">
          <div class="text-h8" style="text-align:center; font-weight:bold">Preconditions</div>
        </q-card-section>
        <div>
          <br />
          <br />
          <p style="font-weight:bold; text-align:center"> No precondition available</p>
          <br />
          <br />

        </div>

        <q-card-actions vertical>
          <q-btn icon="o_photo_camera" color="blue-grey" glossy label="Capture">
            <q-tooltip class="bg-accent">capture goal state</q-tooltip>
          </q-btn>
        </q-card-actions>
      </q-card>

    </div>
    <div class="q-mt-md flex flex-center">
      <q-btn disable icon="o_done_all" color="teal-9" glossy label="Validate" @click="capture()">
      </q-btn>
    </div>

  </div> -->
</template>

<script>
import { defineComponent } from 'vue'
//   import { matMenu } from '@quasar/extras/material-icons'

// export default defineComponent({
//   name: 'IndexPage'
// })


// for table - list of Demonstration
const columns = [
  // {
  //   name: 'index',
  //   label: '#',
  //   field: 'index'
  // },
  {
    name: 'actions',
    required: true,
    label: 'Actions',
    align: 'center',
    field: row => row.name,
    format: val => `${val}`,
    sortable: true
  },
  // {
  //   name: 'createdon',
  //   align: 'center',
  //   label: 'Created On',
  //   field: 'createdon',
  //   sortable: true
  // },
  // {
  //   name: 'solved',
  //   align: 'center',
  //   label: 'Solved',
  //   field: 'solved',
  //   sortable: true
  // },
  // {
  //     name: 'effects',
  //     align: 'center',
  //     label: 'Effects',
  //     field: 'effects',
  //     sortable: false
  // },
]

const seed = [
  {
    name: 'Problem 1',
    createdon: '10/12/2002',
    solved: 'Yes',
    // effects: 'is_open ?g',
  },
]

// we generate lots of rows here
let rows = []
for (let i = 0; i < 1; i++) {
  rows = rows.concat(seed.slice(0).map(r => ({ ...r })))
}
rows.forEach((row, index) => {
  row.index = index
})

// for table Actions
const actionColumns = [
  {
    name: 'index',
    label: '#',
    field: 'index'
  },
  {
    name: 'action',
    required: true,
    label: 'Action',
    align: 'center',
    field: actionRow => actionRow.name,
    format: val => `${val}`,
    sortable: true
    // style: 'max-width: 50px',
    // headerClasses: 'bg-primary text-white',
    // headerStyle: 'max-width: 50px'
  },
]

const actionSeed = [
  {
    name: 'Gripper_go_down(?x, gripper_open, gripper...)',
  },
  {
    name: 'Action 1 (P1, P1)',
  },
  {
    name: 'Action 3 (null, null)',
  },
]

// we generate lots of rows here
let actionRows = []
for (let i = 0; i < 1; i++) {
  actionRows = actionRows.concat(actionSeed.slice(0).map(r => ({ ...r })))
}
actionRows.forEach((actionRows, index) => {
  actionRows.index = index
})


export default {
  defineComponent() {
    name: 'OnProblemValidation'
  },
  setup() {
    return {
      columns,
      rows,
      actionColumns,
      actionRows,
      // pagination: ref({
      // rowsPerPage: 0
      // })
    }
  }
}
</script>
<style lang="sass" scoped>
.my-card
  width: 100%
  max-width: 800px
</style>

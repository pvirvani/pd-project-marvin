<template>
  <div class=" q-pa-md q-gutter-y-md column items-start flex"
       style="background-color:bisque">
    <p style="font-size:20px"><b>Project 1</b></p>
  </div>
  <br />
  <br />

  <div class=" q-pa-md q-gutter-y-md column items-start flex"
       style="float: left; background-color:antiquewhite">
    <q-btn to="/demo"
           color="primary"
           push
           label="Demonstration"
           icon="o_precision_manufacturing"
           size='40px'
           style="width:350px; font-size:30px" />

    <q-btn to="/actions"
           color="primary"
           push
           label="Actions"
           icon="o_question_mark"
           size='40px'
           style="width:350px; font-size:30px" />
    <q-btn to="domain"
           color="primary"
           push
           label="Domain"
           icon="o_domain"
           size='40px'
           style="width:350px; font-size:30px" />
    <q-btn to="/problems"
           color="primary"
           push
           label="Problems"
           icon="o_rule"
           size='40px'
           style="width:350px; font-size:30px" />
  </div>

  <div>


    <!-- table starts from here -->
    <div style="background-color:rosybrown; float:auto"
         class="q-pa-md">
      <q-table title="List of Problems"
               :rows="rows"
               :columns="columns"
               row-key="name"
               virtual-scroll
               v-model:pagination="pagination"
               :rows-per-page-options="0" />
      <div class="q-pa-md q-gutter-sm">
        <q-btn icon="o_add"
               color="primary"
               label="ADD" />
        <q-btn icon="o_file_open"
               color="primary"
               label="OPEN" />
        <q-btn icon="content_copy"
               color="primary"
               label="Copy" />
        <q-btn icon="o_drive_file_rename_outline"
               color="primary"
               label="RENAME" />
        <q-btn icon="o_delete"
               color="red"
               label="DELETE" />
        &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;
        <q-btn icon="o_play_arrow"
               color="primary"
               label="RUN" />
      </div>
    </div>

    <!-- </q-page> -->
    <div style="background-color:white">
      <p>
        <br>
        <br>
      </p>
    </div>


    <div style="background-color:wheat; float:auto"
         class="q-pa-md">
      <div class="q-pa-md row items-start q-gutter-md flex flex-center">
        <!-- card1 -->
        <q-card class="my-card">
          <q-card-section class="bg-grey-8 text-white">
            <div class="text-h6"
                 style="text-align:center">Initial State</div>
          </q-card-section>
          <q-separator />
          <q-card-section class="bg-grey-6 text-white">
            <div class="text-h8"
                 style="text-align:center; font-weight:bold">Visual</div>
          </q-card-section>
          <div>
            <!-- <br />
          <br />


          <p style="font-weight:bold; text-align:center"> Nothing to show</p>
          <br />
          <br /> -->
            <q-img src="../assets/images/state_example.png" />

          </div>
          <q-card-section class="bg-grey-6 text-white">
            <div class="text-h8"
                 style="text-align:center; font-weight:bold">States</div>
          </q-card-section>
          <div>
            <br />
            <br />

            <p style="font-weight:bold; text-align:center"> No state available</p>
            <br />
            <br />


          </div>

          <q-card-actions vertical>
            <q-btn icon="o_edit"
                   color="blue-grey"
                   glossy
                   label="Edit Initial State">
              <q-tooltip class="bg-accent">modify initial state</q-tooltip>
            </q-btn>
          </q-card-actions>
        </q-card>

        <!-- card2 -->
        <q-card class="my-card">
          <q-card-section class="bg-grey-8 text-white">
            <div class="text-h6"
                 style="text-align:center">Goal State</div>
          </q-card-section>
          <q-separator />
          <q-card-section class="bg-grey-6 text-white">
            <div class="text-h8"
                 style="text-align:center; font-weight:bold">Visual</div>
          </q-card-section>
          <div>
            <!-- <br />
          <br />
          <p style="font-weight:bold; text-align:center"> Nothing to show</p>
          <br />
          <br /> -->
            <q-img src="../assets/images/state_example.png" />

          </div>
          <q-card-section class="bg-grey-6 text-white">
            <div class="text-h8"
                 style="text-align:center; font-weight:bold">States</div>
          </q-card-section>
          <div>
            <br />
            <br />
            <p style="font-weight:bold; text-align:center"> No state available</p>
            <br />
            <br />

          </div>

          <q-card-actions vertical>
            <q-btn icon="o_edit"
                   color="blue-grey"
                   glossy
                   label="Edit Goal State">
              <q-tooltip class="bg-accent">edit goal state</q-tooltip>
            </q-btn>
          </q-card-actions>
        </q-card>

      </div>
      <div class="q-mt-md flex flex-center">
        <q-btn to="/problemvalidation"
               icon="o_done_all"
               color="teal-9"
               glossy
               label="Validate"
               @click="capture()">
          <!-- <q-tooltip class="bg-accent">Capture all the States to Enable</q-tooltip> -->
        </q-btn>
      </div>

    </div>
  </div>
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
    name: 'name',
    required: true,
    label: 'Name',
    align: 'center',
    field: row => row.name,
    format: val => `${val}`,
    sortable: true
  },
  {
    name: 'createdon',
    align: 'center',
    label: 'Created On',
    field: 'createdon',
    sortable: true
  },
  {
    name: 'solved',
    align: 'center',
    label: 'Solved',
    field: 'solved',
    sortable: true
  },
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
  row.index = index + 1
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
    name: 'OnOpenProblems'
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
  max-width: 700px
</style>

<template>
  <div class=" q-pa-md q-gutter-y-md column items-start flex"
       style="background-color:bisque">
    <p style="font-size:20px"><b>Project 1</b></p>
  </div>
  <br />
  <br />
  <!-- buttons -->
  <!-- <div class=" q-pa-md q-gutter-y-md column items-start flex"
       style="float: left; background-color:antiquewhite">
    <q-btn to="/demo"
           color="primary"
           push
           label="Demonstration"OnProblems
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
  </div> -->
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
    <q-btn to="/problems"
           color="primary"
           push
           label="Problems"
           icon="o_rule"
           size='40px'
           style="width:350px; font-size:30px" />
    <br />
    <br />
    <br />
    <br />
    <br />
    <br />
    <br />
    <br />
    <br />
    <br />
    <br />
    <br />
    <br />
    <br />
    <br />
    <br />
    <br />
    <br />
    <br />
    <br />
    <br />
    <br />
    <br />
    <br />
    <br />
    <br />
    <br />
    <br />
    <br />
    <br />
    <br />
    <br />
    <q-btn to="domain"
           color="primary"
           push
           label="Domain"
           icon="o_domain"
           size='40px'
           style="width:350px; font-size:30px" />
  </div>


  <div class=" q-pa-md q-gutter-y-md column items-start flex"
       style="float: auto;background-color:antiquewhite">

    <!-- table starts from here -->
    <div style="background-color:rosybrown"
         class="q-pa-md">
      <q-table title="List of Problems"
               :rows="rows"
               :columns="columns"
               row-key="name"
               virtual-scroll
               v-model:pagination="pagination"
               :rows-per-page-options="0" />
      <div class="q-pa-md q-gutter-sm">
        <!-- <q-btn color="primary" label="ADD" />
       -->

        <q-btn color="primary"
               icon="o_add"
               label="add"
               @click="prompt" />

        <q-btn color="primary"
               to="/openproblems"
               icon="o_file_open"
               label="open" />
        <q-btn color="primary"
               icon="o_copy"
               label="copy" />
        <q-btn color="primary"
               icon="o_drive_file_rename_outline"
               label="rename" />
        <q-btn color="red"
               icon="o_delete"
               label="delete" />
      </div>
    </div>

    <!-- </q-page> -->
    <div style="background-color:white">
      <p>
        <br>
        <br>
      </p>
    </div>


    <div style="background-color:wheat"
         class="q-pa-md">
      <p style="text-align:center; font-size:20px"> Please Open a Problem to view its contents</p>

    </div>
  </div>
</template>

<script>
import { defineComponent } from 'vue'
import { useQuasar } from 'quasar'
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
  {
    name: 'Problem 2',
    createdon: '10/12/2012',
    solved: 'No',
    // effects: 'is_open ?g',
  },
  {
    name: 'Problem 3',
    createdon: '10/12/2013',
    solved: 'No',
    // effects: 'is_open ?g',
  },
  {
    name: 'Problem 4',
    createdon: '10/12/2015',
    solved: 'Yes',
    // effects: 'is_open ?g',
  },
  {
    name: 'Problem 5',
    createdon: '10/12/2018',
    solved: 'Yes',
    // effects: 'is_open ?g',
  },
  {
    name: 'Problem 8',
    createdon: '10/12/2020',
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
  actionRows.index = index + 1
})


export default {
  defineComponent() {
    name: 'OnProblems'
  },
  setup() {

    const $q = useQuasar()

    return {
      columns,
      rows,
      actionColumns,
      actionRows,
      // pagination: ref({
      // rowsPerPage: 0
      // })

      prompt() {
        $q.dialog({

          title: 'Prompt',
          message: 'Enter Problem Name:',
          prompt: {
            model: '',
            isValid: val => val.length > 2,
            type: 'text'
          },
          cancel: true,
          persistent: true
        }).onOk(data => {
          console.log('>>>>> Project  ' + data + '  Created')
        })
      },
    }
  }
}
</script>

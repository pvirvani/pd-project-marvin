<template>
  <!-- table starts from here -->
  <!-- <div class="q-pa-md q-gutter-y-md column items-start flex flex-center">

    <q-btn-group push>
      <q-btn to="/demo" color="amber-2" glossy text-color="black" push label="Demo" icon="precision_manufacturing"
        size='50px' />
      <q-btn to="/actions" color="amber-4" glossy text-color="black" push label="Actions" icon="question_mark"
        size='50px' />
      <q-btn to="domain" color="amber-6" glossy text-color="black" push label="Domain" icon="domain" size='50px' />
      <q-btn to="/problems" color="amber-8" glossy text-color="black" push label="Problems" icon="rule" size='50px' />
    </q-btn-group>
  </div> -->
  <div class=" q-pa-md q-gutter-y-md column items-start flex"
       style="background-color:bisque">
    <p style="font-size:20px"><b>Project 1</b></p>
  </div>
  <br />
  <br />
  <div>
    <!-- buttons -->
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

    <!-- List of Demonstrations -->
    <div class=" q-pa-md q-gutter-y-md row items-start flex"
         style="float: left;background-color:antiquewhite">
      <div style="background-color:rosybrown; float:left"
           class="q-pa-md">
        <!-- <q-table ref="tableRef"
                 title="List of Demonstrations"
                 :rows="rows"
                 :columns="columns"
                 row-key="name"
                 selection="multiple"
                 v-model:selected="selected" /> -->
        <q-table ref="tableRef"
                 title="Projects"
                 :rows="rows"
                 :columns="columns"
                 row-key="name"
                 selection="multiple"
                 v-model:selected="selected">
          <template v-slot:header-selection="scope">
            <q-checkbox v-model="scope.selected" />
          </template>

          <template v-slot:body-selection="scope">
            <q-checkbox :model-value="scope.selected"
                        @update:model-value="(val, evt) => { Object.getOwnPropertyDescriptor(scope, 'selected').set(val, evt) }" />
          </template>
        </q-table>
        <div class="q-pa-md q-gutter-sm">
          <!-- <q-btn color="primary" label="ADD" /> -->
          <q-btn color="primary"
                 icon="o_add"
                 label="add"
                 @click="prompt_add" />

          <q-btn color="primary"
                 icon="o_file_open"
                 label="open" />

          <q-btn color="primary"
                 icon="o_copy"
                 label="copy" />

          <q-btn color="primary"
                 icon="o_drive_file_rename_outline"
                 label="rename"
                 @click="prompt_rename" />

          <q-btn color="red"
                 icon="o_delete"
                 label="delete"
                 @click="confirm_delete" />
        </div>
      </div>
      &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;
      <!-- Actions to be added or removed -->
      <div style="background-color:wheat; float:right"
           class="q-pa-md">
        <p style="font-size:1.2vw"> Actions </p>
        <p style="font-size:1vw"> Add, Remove, Rearrange actions of the demonstration </p>

        <div class="q-pa-md q-gutter-y-md column items-start flex flex-center">
          <q-btn class="fit"
                 color="blue-3"
                 glossy
                 text-color="black"
                 push
                 label="Add Actions"
                 icon="add"
                 size="20px"
                 padding="xs lg">
            <q-menu class="scrollable">
              <q-list dense
                      style="min-width: 20px">
                <q-item clickable
                        v-close-popup>
                  <q-item-section>Pick_Up(?x1, block_size, color, position)</q-item-section>
                </q-item>
                <q-item clickable
                        v-close-popup>
                  <q-item-section>Go_left(null, null)</q-item-section>
                </q-item>

                <q-item clickable
                        v-close-popup>
                  <q-item-section>Hold(null, x)</q-item-section>
                </q-item>

                <q-item clickable
                        v-close-popup>
                  <q-item-section>push_down(P1, P1)</q-item-section>
                </q-item>

                <q-item clickable
                        v-close-popup>
                  <q-item-section>A2(P1, P1)</q-item-section>
                </q-item>

                <q-item clickable
                        v-close-popup>
                  <q-item-section>Go_right(P1, P1)</q-item-section>
                </q-item>

                <q-separator />
                <q-item clickable
                        v-close-popup>
                  <q-item-section>Quit</q-item-section>
                </q-item>
              </q-list>

            </q-menu>
          </q-btn>
        </div>
        <div style="background-color:rosybrown"
             class="q-pa-md">
          <q-table :rows="actionRows"
                   :columns="actionColumns"
                   row-key="name" />
        </div>

        <div class="q-pa-md ">
          <!-- <q-btn color="green" icon="o_save" label="Save" glossy></q-btn> -->
          <q-btn :loading="progress[0].loading"
                 :percentage="progress[0].percentage"
                 color="primary"
                 @click="startComputing(0)"
                 style="width: 150px">
            Save
            <template v-slot:loading>
              <q-spinner-gears class="on-left" />
              Saving...
            </template>
          </q-btn>
        </div>
      </div>
    </div>
  </div>
  <!-- </q-page> -->

</template>

<script>
import { defineComponent, onBeforeUnmount, ref } from 'vue'
import { computed, toRaw, nextTick } from 'vue'
import { date } from 'quasar'
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
    align: 'left',
    field: row => row.name,
    format: val => `${val}`,
    sortable: true
  },
  {
    name: 'createdOn',
    align: 'center',
    label: 'Created On',
    field: 'createdOn',
    sortable: true
  },
  {
    name: 'totalActions',
    label: 'Total Actions',
    field: 'totalActions',
    sortable: true
  },
]

const seed = [
  {
    name: 'demo1',
    createdOn: '30/03/2022',
    totalActions: 6,
  },
  {
    name: 'demo2',
    createdOn: '30/03/2022',
    totalActions: 9,
  },
  {
    name: 'test1',
    createdOn: '30/03/2022',
    totalActions: 16,
  },
  {
    name: 'testDemo',
    createdOn: '30/03/2022',
    totalActions: 3,
  },
  {
    name: 'Gingerbread',
    createdOn: '30/03/2022',
    totalActions: 16,

  },
  {
    name: 'Jelly bean',
    createdOn: '30/03/2022',
    totalActions: 0,

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
  // {
  //   name: 'index',
  //   label: '#',
  //   field: 'index'
  // },
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
    name: 'OnDemonstration'
  },
  setup() {
    const progress = ref([
      { loading: false, percentage: 0 },
      { loading: false, percentage: 0 },
      { loading: false, percentage: 0 }
    ])

    const intervals = [null, null, null]

    function startComputing(id) {
      progress.value[id].loading = true
      progress.value[id].percentage = 0

      intervals[id] = setInterval(() => {
        progress.value[id].percentage += Math.floor(Math.random() * 8 + 10)
        if (progress.value[id].percentage >= 100) {
          clearInterval(intervals[id])
          progress.value[id].loading = false
        }
      }, 700)
    }

    onBeforeUnmount(() => {
      intervals.forEach(val => {
        clearInterval(val)
      })
    })

    const $q = useQuasar()
    const tableRef = ref()
    const selected = ref([])


    return {
      columns,
      rows,
      actionColumns,
      actionRows,
      tableRef,
      selected,

      // pagination: ref({
      // rowsPerPage: 0
      // })
      progress,
      startComputing,

      // prompt() {
      //   $q.dialog({

      //     title: 'Prompt',
      //     message: 'Enter Demonstration Name:',
      //     prompt: {
      //       model: '',
      //       isValid: val => val.length > 2,
      //       type: 'text'
      //     },
      //     cancel: true,
      //     persistent: true
      //   }).onOk(data => {
      //     console.log('>>>>> Project  ' + data + '  Created')
      //   })
      // },
      addRow() {
        loading.value = true
        setTimeout(() => {
          const
            index = Math.floor(Math.random() * (rows.value.length + 1)),
            row = originalRows[Math.floor(Math.random() * originalRows.length)]

          if (rows.value.length === 0) {
            rowCount.value = 0
          }

          row.id = ++rowCount.value
          const newRow = { ...row } // extend({}, row, { name: `${row.name} (${row.__count})` })
          rows.value = [...rows.value.slice(0, index), newRow, ...rows.value.slice(index)]
          loading.value = false
        }, 500)
      },

      removeRow() {
        loading.value = true
        setTimeout(() => {
          const index = Math.floor(Math.random() * rows.value.length)
          rows.value = [...rows.value.slice(0, index), ...rows.value.slice(index + 1)]
          loading.value = false
        }, 500)
      },

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
          console.log('>>>>> Project  ' + data + '  Created')
        })
      },

      confirm_delete() {
        $q.dialog({
          title: 'Confirm',
          message: 'Do you want to delete the demonstration?',
          cancel: true,
          persistent: true
        }).onOk(() => {
          // console.log('>>>> OK')
        }).onOk(() => {
          // console.log('>>>> second OK catcher')
        }).onCancel(() => {
          // console.log('>>>> Cancel')
        }).onDismiss(() => {
          // console.log('I am triggered on both OK and Cancel')
        })
      },

      prompt_rename() {
        $q.dialog({
          title: 'Prompt_Rename',
          message: 'Enter new name:',
          prompt: {
            model: '',
            isValid: val => val.length > 2,
            type: 'text'
          },
          cancel: true,
          persistent: true
        }).onOk(data => {
          console.log('>>>>> Project  ' + data + '  Renamed')
        })
      },


      handleSelection({ rows, added, evt }) {
        // ignore selection change from header of not from a direct click event
        if (rows.length !== 1 || evt === void 0) {
          return
        }

        const oldSelectedRow = storedSelectedRow
        const [newSelectedRow] = rows
        const { ctrlKey, shiftKey } = evt

        if (shiftKey !== true) {
          storedSelectedRow = newSelectedRow
        }

        // wait for the default selection to be performed
        nextTick(() => {
          if (shiftKey === true) {
            const tableRows = tableRef.value.filteredSortedRows
            let firstIndex = tableRows.indexOf(oldSelectedRow)
            let lastIndex = tableRows.indexOf(newSelectedRow)

            if (firstIndex < 0) {
              firstIndex = 0
            }

            if (firstIndex > lastIndex) {
              [firstIndex, lastIndex] = [lastIndex, firstIndex]
            }

            const rangeRows = tableRows.slice(firstIndex, lastIndex + 1)
            // we need the original row object so we can match them against the rows in range
            const selectedRows = selected.value.map(toRaw)

            selected.value = added === true
              ? selectedRows.concat(rangeRows.filter(row => selectedRows.includes(row) === false))
              : selectedRows.filter(row => rangeRows.includes(row) === false)
          }
          else if (ctrlKey !== true && added === true) {
            selected.value = [newSelectedRow]
          }
        })
      },



    }// return
  }//setup
}//export
</script>

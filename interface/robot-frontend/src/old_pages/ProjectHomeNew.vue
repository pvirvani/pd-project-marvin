<template>

  <!-- <div class="q-pa-md" style=" max-width: 400px">

    <q-form @submit="onSubmit" @reset="onReset" class="q-gutter-md">
      <q-input filled v-model="name" label="Search By Name" hint="Enter project name to be searched" lazy-rules
        :rules="[ val => val && val.length > 0 || 'Please type something']" />

      <div>
        <q-btn label="Search" type="submit" color="primary" />
      </div>
    </q-form>

  </div> -->
  <!-- </div> -->

  <!-- table starts from here -->

  <div class="q-pa-md">
    <!-- <q-table title="Projects" :rows="rows" :columns="columns" row-key="name" virtual-scroll
      v-model:pagination="pagination" :rows-per-page-options="0" /> -->

    <!-- <div class="q-pa-md">
      <q-table
        title="Projects"
        :rows="rows"
        :columns="columns"
        row-key="id"
        :filter="filter"
        :loading="loading"
        selection="multiple"
        v-model:selected="selected">

        <template v-slot:top>
          <q-btn
            color="primary"
            :disable="loading"
            label="Add row"
            @click="addRow" />
          <q-btn
            class="q-ml-sm"
            color="primary"
            :disable="loading"
            label="Remove row"
            @click="removeRow" />
          <q-space />
         <q-input
            dense
            debounce="300"
            color="primary"
            v-model="filter">
           <template
              v-slot:append>
             <q-icon
                name="search" />
            </template>
          </q-input>
        </template>

      </q-table>
    </div> -->


    <div class="q-pa-md">
      <!-- <div class="text-subtitle1 q-pa-sm">Use <kbd>SHIFT</kbd> to select / deselect a range and <kbd>CTRL</kbd> to add to selection</div> -->

      <!-- <q-table
      ref="tableRef"
      title="Treats"
      :rows="rows"
      :columns="columns"
      row-key="name"
      selection="multiple"
      v-model:selected="selected"
      @selection="handleSelection"
    > -->

      <!-- table with multiple selections -->

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
  </div>
  <!-- for date class -->
  <!-- <div class="q-pa-md" style="max-width: 200px">
    <q-input filled v-model="input" mask="date" :rules="['date']">
      <template v-slot:append>
        <q-icon name="event" class="cursor-pointer">
          <q-popup-proxy cover :breakpoint="600">
            <q-date v-model="input" />
          </q-popup-proxy>
        </q-icon>
      </template>
    </q-input>
  </div> -->

  <!-- </q-page> -->
</template>

<script>
import { defineComponent, ref, computed, toRaw, nextTick } from 'vue'
import { date } from 'quasar'
import { useQuasar } from 'quasar'

//   import { matMenu } from '@quasar/extras/material-icons'

// export default defineComponent({
//   name: 'IndexPage'
// })


// for table
const columns = [
  // {
  //   name: 'index',
  //   label: 'Project_ID',
  //   field: 'index',
  //   align: 'center'
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
    // format: val => `${val}`,
    sortable: true
  },
  {
    name: 'totalActions',
    label: 'Total Actions',
    field: 'totalActions',
    sortable: true
  },
  {
    name: 'totalProblems',
    label: 'Total Problems',
    field: 'totalProblems',
    sortable: true
  },
]

const seed = [
  {
    name: 'project1',
    createdOn: 30032022,
    totalActions: 6,
    totalProblems: 5,
  },
  {
    name: 'project2',
    createdOn: 30032022,
    totalActions: 9,
    totalProblems: 8,
  },
  {
    name: 'Eclair',
    createdOn: 30032022,
    totalActions: 16,
    totalProblems: 7,
  },
  {
    name: 'Cupcake',
    createdOn: 30032022,
    totalActions: 6,
    totalProblems: 2,
  },
  {
    name: 'Gingerbread',
    createdOn: 30032022,
    totalActions: 16,
    totalProblems: 5,
  },
  {
    name: 'Jelly bean',
    createdOn: 30032022,
    totalActions: 4,
    totalProblems: 0,

  },
]

// we generate lots of rows here
// let rows = []
// for (let i = 0; i < 1; i++) {
//   rows = rows.concat(seed.slice(0).map(r => ({ ...r })))
// }
// rows.forEach((row, index) => {
//   row.index = index + 1
// })


let originalRows = []
for (let i = 0; i < 1; i++) {
  originalRows = originalRows.concat(seed.slice(0).map(r => ({ ...r })))
}
originalRows.forEach((originalRow, index) => {
  originalRow.index = index + 1
})

// // Projects Variable
// let id = 0
// const newproject = ref('')
// const projects = ref([
//   { id: id++, text: 'Learn HTML' },
//   { id: id++, text: 'Learn JavaScript' },
//   { id: id++, text: 'Learn Vue' }
// ])

// // add project function
// function addProject() {
//   projects.value.push({ id: id++, text: newproject.value })
//   newproject.value = ''
// }


export default {
  defineComponent() {
    name: 'ProjectHomeNew'
  },
  setup() {

    const $q = useQuasar()
    const loading = ref(false)
    const filter = ref('')
    const rowCount = ref(10)
    const rows = ref([...originalRows])
    const hasData = ref(true)
    const tableRef = ref()
    const selected = ref([])
    let storedSelectedRow

    return {
      columns,
      rows,
      tableRef,
      selected,
      // originalRows,
      // hasData,
      // loading,
      // filter,
      // rowCount,
      // selected: ref([rows[1]]),
      // columns, records: computed(() => hasData.value === true ? rows : [],


      // newproject,
      // input: ref(''),
      // dare: ref('31/10/2022'),
      // pagination: ref({
      // rowsPerPage: 0
      // })
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
          message: 'Do you want to delete the project?',
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
    } // return end
  } // setup end





  // computed: {
  //   dateToday() {
  //     let timeStamp = Date.now()
  //     return date.formatDate(timeStamp, 'DD/MMM/YYYY')
  //   },
  //   // currentVersion() {
  //   //   return "Version 1.0"
  //   // }
  // },

} // export end
</script>

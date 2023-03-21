<template>
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

  <!-- buttons -->
  <!-- <div class=" q-pa-md q-gutter-y-md column items-start flex"
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
      <q-table title="List of Actions"
               :rows="rows"
               :columns="columns"
               row-key="name"
               virtual-scroll
               v-model:pagination="pagination"
               :rows-per-page-options="0" />
      <div class="q-pa-md q-gutter-sm">
        <!-- <q-btn color="primary" label="ADD" /> -->
        <q-btn color="primary"
               icon="o_add"
               label="add"
               @click="prompt" />

        <!-- <q-btn color="primary" icon="o_file_open" label="open" /> -->
        <!-- <q-btn color="primary" icon="o_copy" label="copy" /> -->
        <q-btn color="primary"
               icon="o_tune"
               label="edit" />
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
      <!-- <p style="font-size:2vw"> Action Name </p> -->
      <div class="q-pa-md"
           style="max-width: 400px">
        <p style="font-size:24px">Action Name</p>
        <q-form @submit="onSubmit"
                @reset="onReset"
                class="q-gutter-md">
          <q-input filled
                   v-model="name"
                   label="Type Action Name *"
                   lazy-rules
                   :rules="[val => val && val.length > 0 || 'Please type action name']" />
          <div class="q-pa-md q-gutter-y-md column flex flex-end items-end">
            <q-btn color="blue-3"
                   glossy
                   text-color="black"
                   push
                   label="Demonstrate"
                   icon="precision_manufacturing"
                   size="20px"
                   padding="xs lg" />
          </div>
          <!-- <q-btn label="Submit" type="submit" color="primary" /> -->
          <!-- <q-btn label="Reset" type="reset" color="primary" flat class="q-ml-sm" /> -->
        </q-form>

      </div>
      <!-- cards -->
      <div class="q-pa-md row items-start q-gutter-md flex flex-center"
           style="max-width: 2000px">
        <!-- parameter card -->
        <q-card class="my-card">

          <q-card-section class="bg-primary text-white">
            <div class="text-h6">Paramerers</div>
            <div class="text-subtitle2"></div>
          </q-card-section>

          <q-separator />

          <q-card-actions align="right">
            <!-- <q-btn icon="addition">Add New Parameter</q-btn> -->
            <div class="q-pa-md q-gutter-y-md column flex flex-end items-end">
              <q-btn-dropdown color="blue-3"
                              glossy
                              text-color="black"
                              push
                              label="Add New Parameter"
                              icon="precision_manufacturing"
                              size="20px"
                              padding="xs lg">
                <q-list>
                  <q-item clickable
                          v-close-popup>
                    <q-item-section>
                      <q-item-label>
                        ?g
                        <!-- <q-checkbox v-model="val" /> -->
                      </q-item-label>
                    </q-item-section>
                  </q-item>

                  <q-item clickable
                          v-close-popup>
                    <q-item-section>
                      <q-item-label>Gripper ?x</q-item-label>
                    </q-item-section>
                  </q-item>

                  <q-item clickable
                          v-close-popup>
                    <q-item-section>
                      <q-item-label>Block_2x6 ?d</q-item-label>
                    </q-item-section>
                  </q-item>
                </q-list>

              </q-btn-dropdown>
            </div>
          </q-card-actions>
        </q-card>

        <!-- preconditions card -->
        <q-card class="my-card">
          <q-card-section class="bg-purple text-white">
            <div class="text-h6">Preconditions</div>
            <div class="text-subtitle2"></div>
          </q-card-section>

          <q-separator />

          <q-card-actions align="right">
            <!-- <q-btn flat>Action 1</q-btn> -->
            <!-- <q-btn flat>Action 2</q-btn> -->
            <div class="q-pa-md q-gutter-y-md column flex flex-end items-end">
              <q-btn-dropdown color="blue-3"
                              glossy
                              text-color="black"
                              push
                              label="Add New Precondition"
                              icon="question_mark"
                              size="20px"
                              padding="xs lg">

                <q-list>
                  <q-item clickable
                          v-close-popup>
                    <q-item-section>
                      <q-item-label>not(gripper_vertical ?g)</q-item-label>
                    </q-item-section>
                  </q-item>

                  <q-item clickable
                          v-close-popup>
                    <q-item-section>
                      <q-item-label>(held_from_left ?x)</q-item-label>
                    </q-item-section>
                  </q-item>

                  <q-item clickable
                          v-close-popup>
                    <q-item-section>
                      <q-item-label>unloaded ?g</q-item-label>
                    </q-item-section>
                  </q-item>
                </q-list>

              </q-btn-dropdown>
            </div>
          </q-card-actions>
        </q-card>

        <!-- preconditions card -->
        <q-card class="my-card"
                style="max-width: 700px">
          <q-card-section class="bg-deep-orange text-white">
            <div class="text-h6"> Effects
            </div>
            <div class="text-subtitle2"></div>
          </q-card-section>

          <q-separator />

          <q-card-actions align="right">
            <!-- <q-btn flat>Action 1</q-btn> -->
            <!-- <q-btn flat>Action 2</q-btn> -->
            <div class="q-pa-md q-gutter-y-md column flex flex-end items-end">
              <q-btn-dropdown color="blue-3"
                              glossy
                              text-color="black"
                              push
                              label="Add New Effect"
                              icon="o_hive"
                              size="20px"
                              padding="xs lg">
                <q-list>
                  <q-item clickable
                          v-close-popup>
                    <q-item-section>
                      <q-item-label>is_open ?g</q-item-label>
                    </q-item-section>
                  </q-item>

                  <q-item clickable
                          v-close-popup>
                    <q-item-section>
                      <q-item-label>is_close ?g</q-item-label>
                    </q-item-section>
                  </q-item>

                  <q-item clickable
                          v-close-popup>
                    <q-item-section>
                      <q-item-label>is_held ?g</q-item-label>
                    </q-item-section>
                  </q-item>
                </q-list>

              </q-btn-dropdown>
            </div>
          </q-card-actions>
        </q-card>

      </div>
      <!-- save button -->
      <div class="q-pa-md q-gutter-y-md column flex flex-end items-end">
        <!-- <q-btn color="green" glossy text-color="black" push label="Save" icon="save" size="20px" padding="xs lg" /> -->
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
</template>

<script>
import { defineComponent, onBeforeUnmount } from 'vue';
import { ref } from 'vue';
import { useQuasar } from 'quasar'
//   import { matMenu } from '@quasar/extras/material-icons'

// export default defineComponent({
//   name: 'IndexPage'
// })


// for table - list of Demonstration
const columns = [
  {
    name: 'index',
    label: '#',
    field: 'index'
  },
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
    name: 'parameters',
    align: 'center',
    label: 'Parameters',
    field: 'parameters',
    sortable: false
  },
  {
    name: 'preconditions',
    align: 'center',
    label: 'Preconditions',
    field: 'preconditions',
    sortable: false
  },
  {
    name: 'effects',
    align: 'center',
    label: 'Effects',
    field: 'effects',
    sortable: false
  },
]

const seed = [
  {
    name: 'Open_gripper',
    parameters: '?g - Gripper',
    preconditions: 'unloaded ?g',
    effects: 'is_open ?g',
  },
  {
    name: 'Hold_gripper',
    parameters: '?x - Block ?g - Gripper',
    preconditions: 'is_open ?g, unloaded ?g',
    effects: 'loaded ?g ?x, not(is_open ?g),...',
  },
  {
    name: 'Hold_gripper_left',
    parameters: '?x - Block ?g - Gripper',
    preconditions: 'is_open ?g, unloaded ?g',
    effects: 'loaded ?g ?x, not(is_open ?g),...',
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
    name: 'OnActions'
  },
  setup() {

    const $q = useQuasar()

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
    return {
      columns,
      rows,
      actionColumns,
      actionRows,
      val: ref(true),
      progress,
      startComputing,
      // pagination: ref({
      // rowsPerPage: 0
      // })

      prompt() {
        $q.dialog({

          title: 'Prompt',
          message: 'Enter Action Name:',
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
    }//retun
  }//setup
}//export
</script>


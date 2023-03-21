<template>
    <q-page class="flex flex-center">
        <div class="q-mt-md q-gutter-md">
            <div>
                <!-- <q-btn color="red" icon-right="send" label="On Left and Right" /> -->
                <q-btn-group spread>
                    <q-btn color="amber-2"
                           push>
                        <div class="row items-center no-wrap">
                            <q-icon right
                                    name="sym_o_create_new_folder"
                                    color="primary"
                                    size="200px" />
                            <div class="text-center">
                                <p style="color:darkcyan;font-size:22px;position: relative;">Create New Project</p>
                            </div>
                        </div>
                    </q-btn>
                    <q-btn color="amber-3"
                           push>
                        <div class="row items-center no-wrap">
                            <q-icon right
                                    name="sym_o_folder_open"
                                    color="primary"
                                    size="200px" />
                            <div class="text-center">
                                <p style="color:darkcyan;font-size:22px;position: relative;">Open Project</p>
                            </div>
                        </div>
                    </q-btn>
                </q-btn-group>
            </div>
        </div>
    </q-page>
</template>
  
<script>
import { defineComponent } from 'vue'
//   import { matMenu } from '@quasar/extras/material-icons'

// export default defineComponent({
//   name: 'IndexPage'
// })

export default {
    defineComponent() {
        name: 'ProjectHome'
    },
    //   created() {
    //       this.matMenu = matMenu
    //   }
}
</script>
  
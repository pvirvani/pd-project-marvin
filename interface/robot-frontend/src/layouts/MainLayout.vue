<template>
  <q-layout view="lHh Lpr lFf">
    <q-header elevated>
      <q-toolbar class="glossy bg-purple">
        <q-btn rounded color="red" dense round icon="menu" aria-label="Menu" @click="toggleLeftDrawer" />

        <q-toolbar-title>
          <div style="font-size: 18px">
            <center>ANR PRCE Prog4Yu - YuMi Collaborative Robot</center>
          </div>
        </q-toolbar-title>

        <q-btn outine rounded color="green">
          <!-- <div class="glossy"> -->
          <q-icon left name="sym_o_calendar_today" />
          <div class="text-center">
            {{ dateToday }}
          </div>
          <!-- </div> -->
        </q-btn>

        <q-btn outine rounded color="orange">
          <!-- <div class="glossy"> -->
          <q-icon left name="sym_o_description" />
          <div class="text-center">
            {{ currentVersion }}
          </div>
          <!-- </div> -->
        </q-btn>

        <!-- <div>Quasar v{{ $q.version }}</div> -->
        <!-- <div>Version 1.0</div> -->
      </q-toolbar>

      <!-- <q-tabs v-model="tab">
          <q-tab name="images" label="Images" />
          <q-tab name="videos" label="Videos" />
          <q-tab name="articles" label="Articles" />
        </q-tabs> -->
    </q-header>

    <q-drawer v-model="leftDrawerOpen" show-if-above bordered>
      <q-list>
        <q-item-label header>
          Essential Links
        </q-item-label>

        <EssentialLink v-for="link in essentialLinks" :key="link.title" v-bind="link" />
      </q-list>
    </q-drawer>

    <q-page-container>
      <router-view />
    </q-page-container>
  </q-layout>
</template>

<script>
import { defineComponent, ref } from 'vue'
import EssentialLink from 'components/EssentialLink.vue'
import { date } from 'quasar'

const linksList = [
  {
    title: 'Docs',
    caption: 'quasar.dev',
    icon: 'school',
    link: 'https://quasar.dev'
  },
  {
    title: 'Github',
    caption: 'github.com/quasarframework',
    icon: 'code',
    link: 'https://github.com/quasarframework'
  },
  {
    title: 'Discord Chat Channel',
    caption: 'chat.quasar.dev',
    icon: 'chat',
    link: 'https://chat.quasar.dev'
  },
  {
    title: 'Forum',
    caption: 'forum.quasar.dev',
    icon: 'record_voice_over',
    link: 'https://forum.quasar.dev'
  },
  {
    title: 'Twitter',
    caption: '@quasarframework',
    icon: 'rss_feed',
    link: 'https://twitter.quasar.dev'
  },
  {
    title: 'Facebook',
    caption: '@QuasarFramework',
    icon: 'public',
    link: 'https://facebook.quasar.dev'
  },
  {
    title: 'Quasar Awesome',
    caption: 'Community Quasar projects',
    icon: 'favorite',
    link: 'https://awesome.quasar.dev'
  }
]

export default defineComponent({
  name: 'MainLayout',

  components: {
    EssentialLink
  },

  setup() {
    const leftDrawerOpen = ref(false)

    return {
      essentialLinks: linksList,
      leftDrawerOpen,
      toggleLeftDrawer() {
        leftDrawerOpen.value = !leftDrawerOpen.value
      }
    }
  },

  computed: {
    dateToday() {
      let timeStamp = Date.now()
      return date.formatDate(timeStamp, 'DD/MMM/YYYY')
    },
    currentVersion() {
      return "Version 1.0"
    }
  }
})
</script>

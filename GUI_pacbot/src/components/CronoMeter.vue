<template>
  <div id="chrono">
    <img class="img" src="../assets/cronometro.png" />
    <a class="timer">{{zfill(hour)}}:{{zfill(min)}}:{{zfill(sec)}}</a>
  </div>
</template>

<script lang="js">
export default {
  name: "CronoMeter",
  data() {
    return {
      sec: 0,
      min: 0,
      hour: 0,
      timer: null,
    };
  },
  methods: {
    zfill(number) {
      return number.toString().padStart(2, 0);
    },
    play() {
      if (this.timer === null) {
        this.playing();
        this.timer = setInterval(() => this.playing(), 1000);
      } else {
        clearInterval(this.timer);
        this.timer = null;
        this.pause();
      }
    },
    playing() {
      this.sec++;
      if (this.sec >= 59) {
        this.sec = 0;
        this.min++;
      }
      if (this.min >= 59) {
        this.min = 0;
        this.hour++;
      }
    },
    pause() {
      this.intervalList.push(
        `${this.zfill(this.hour)}:${this.zfill(this.min)}:${this.zfill(
          this.sec
        )}`
      );
      console.log(this.intervalList);
    },
    clear() {
      if (this.timer !== null) {
        clearInterval(this.timer);
        this.timer = null;
      }
      this.sec = 0;
      this.min = 0;
      this.hour = 0;
      this.clearIntervalList();
    },
    clearIntervalList() {
      this.intervalList = [];
      console.log(this.intervalList);
    },
  },
};
</script>

<style>
@import "../style/css/CronoMeter.css";
</style>

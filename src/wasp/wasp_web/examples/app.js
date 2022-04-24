const app = Vue.createApp({
  data() {
    return {
      status: "---",
      param: 0
    }
  },
  mounted() {
    this.ros = new ROSLIB.Ros({
      url: 'ws://localhost:9090'
    });
  },
})
vm = app.mount('#app')

vm.ros.on('connection', () => {
    vm.status = "--connected--";
});
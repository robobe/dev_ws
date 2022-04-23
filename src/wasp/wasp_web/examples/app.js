const app = Vue.createApp({
    data() { 
            return {
                status: "---"
            }
    },
    mounted() {
        this.ros = new ROSLIB.Ros({
          url : 'ws://localhost:9090'
        });
    
        this.ros.on('connection', () => {
          this.status = "--connected--";
        });
      }
})
app.mount('#app')
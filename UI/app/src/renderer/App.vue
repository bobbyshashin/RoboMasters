<template>
  <div id="#app">
    <section class="hero is-dark">
      <nav class="nav has-shadow">
        <div class="container">
          <div class="nav-left">
            <a class="nav-item">
              HKUST Robomasters
            </a>
          </div>
          <span class="nav-toggle">
                  <span></span>
                  <span></span>
                  <span></span>
                </span>
          <div class="nav-right nav-menu">
            <a class="nav-item is-active">
              Graphs
            </a>
          </div>
        </div>
      </nav>
    </section>

    <router-view></router-view>
  </div>
</template>

<script>
  import "bulma/css/bulma.css";

  export default {
  }
</script>

<style>
</style>

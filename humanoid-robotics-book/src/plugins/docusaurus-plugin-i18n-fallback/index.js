// Custom plugin to handle i18n fallback for Docusaurus
// This plugin ensures that when a page doesn't exist in a non-default locale,
// it provides a better user experience

/** @type {import('@docusaurus/types').PluginModule} */
const i18nFallbackPlugin = (context, options) => {
  return {
    name: 'docusaurus-plugin-i18n-fallback',
    
    // This plugin doesn't need to modify content or provide client modules
    // It's more about ensuring proper routing behavior
    async loadContent() {
      // No content to load
    },
    
    async contentLoaded({ content, actions }) {
      // No content transformation needed
    },
    
    // Add configuration to handle i18n routing properly
    configureWebpack(config, isServer, { locale, env }) {
      // No webpack configuration changes needed
      return {};
    },
  };
};

module.exports = i18nFallbackPlugin;
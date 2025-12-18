# How to Start Dev Server with i18n Enabled

## Prerequisites
Make sure you have Node.js and npm/yarn installed on your system.

## Install Dependencies
First, make sure all dependencies are installed:

```bash
npm install
```

## Starting the Development Server

### Option 1: Serve all locales
```bash
npm run start
```
This will start the development server and serve both English and Urdu versions of your site.

### Option 2: Serve a specific locale only
If you want to develop focusing on a specific locale, you can start the server with a specific locale:

For English only:
```bash
npm run start -- --locale en
```

For Urdu only:
```bash
npm run start -- --locale ur
```

## Building and Serving Locally

To build and serve the site locally with all locales:

```bash
npm run build
npm run serve
```

To build only a specific locale:
```bash
npm run build -- --locale ur
```

## Testing the Language Switcher

1. Once the server is running, visit `http://localhost:3000` (English version)
2. You should see the language switcher dropdown in the navbar
3. Click on the language switcher and select "اردو" to view the Urdu version
4. If Urdu translations don't exist for a particular page, it will fallback to English

## Important Notes:

1. When you first start the dev server, it might show warnings about missing translations - this is normal
2. As you add Urdu translations to your `i18n/ur/` directory, those pages will override the English defaults
3. Docusaurus will automatically handle the fallback mechanism when Urdu docs don't exist yet
4. The URL routing follows the pattern:
   - English: `http://localhost:3000/` or `http://localhost:3000/en/`
   - Urdu: `http://localhost:3000/ur/`

## Troubleshooting:

- If the language switcher doesn't appear, verify that `"type": "localeDropdown"` is properly added to your navbar items in `docusaurus.config.js`
- If Urdu content isn't showing, make sure you've created the proper directory structure as outlined in URDU_DOCS_STRUCTURE.md
# How to Translate Navbar Labels to Urdu

To translate navbar labels in your Docusaurus site, you need to create a JSON file that contains the translations. Follow these steps:

## Step 1: Create navbar.json file

Create the following file:
```
i18n/ur/docusaurus-theme-classic/navbar.json
```

## Step 2: Add navbar translations

Add the following content to the navbar.json file:

```json
{
  "title": {
    "message": "ہیومنوائڈ روبوٹکس اور فزیکل AI",
    "description": "Navbar title"
  },
  "item.label.📚 Book": {
    "message": "📚 کتاب",
    "description": "Navbar item with label 📚 Book"
  },
  "item.label.💼 Careers": {
    "message": "💼 کیریئر",
    "description": "Navbar item with label 💼 Careers"
  },
  "item.label.GitHub repository": {
    "message": "GitHub ذخیرہ",
    "description": "Navbar item with label GitHub repository"
  }
}
```

## Step 3: Translating other UI elements

You can also create other translation files for different parts of your site:

### For footer translations:
Create: `i18n/ur/docusaurus-theme-classic/footer.json`
```json
{
  "link.item.label.Foundations": {
    "message": "بنیادیں",
    "description": "Footer link label for Foundations"
  },
  "link.item.label.ROS 2 Basics": {
    "message": "ROS 2 بنیادیں",
    "description": "Footer link label for ROS 2 Basics"
  },
  "link.item.label.Digital Twin": {
    "message": "ڈیجیٹل ٹوئن",
    "description": "Footer link label for Digital Twin"
  },
  "link.item.label.Isaac Platform": {
    "message": "آئزاک پلیٹ فارم",
    "description": "Footer link label for Isaac Platform"
  },
  "copyright": {
    "message": "کاپی رائٹ © ${currentYear} ہیومنوائڈ روبوٹکس اور فزیکل AI بک.",
    "description": "Footer copyright"
  }
}
```

## Important Notes:

1. The keys in these JSON files follow a specific pattern based on how Docusaurus generates the UI elements
2. When you add the localeDropdown type to your navbar, Docusaurus will automatically create the language switching interface
3. The RTL (right-to-left) direction is already configured in docusaurus.config.js for Urdu
4. You can see what UI elements need translation by looking at the English translation files in your `.docusaurus` folder after running the dev server once
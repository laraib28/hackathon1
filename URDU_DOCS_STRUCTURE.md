# Urdu Documentation Folder Structure

For Docusaurus i18n with Urdu language support, here is the exact folder structure:

```
i18n/
└── ur/                           # Urdu locale directory
    ├── docusaurus-plugin-content-docs/
    │   └── current/              # Current version docs (if using versions)
    │       ├── Part1-Foundation/ # Copy of English docs translated to Urdu
    │       ├── Part2-Modules/    # Copy of English docs translated to Urdu
    │       ├── Part3-Capstone/   # Copy of English docs translated to Urdu
    │       └── Part4-Future/     # Copy of English docs translated to Urdu
    └── docusaurus-theme-classic/
        └── navbar.json           # Navbar label translations
```

## File Structure Explained:

1. `i18n/ur/` - Main Urdu locale directory
2. `docusaurus-plugin-content-docs/current/` - Contains translated documentation files
3. `docusaurus-theme-classic/navbar.json` - Contains translations for navbar labels

## How It Works:
- If a document exists in `i18n/ur/docusaurus-plugin-content-docs/current/`, it will be served for the Urdu version
- If a document doesn't exist in Urdu, it will automatically fallback to the English version
- Translated documents should have the exact same filename as their English counterparts
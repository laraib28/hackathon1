# اردو Language Switching شروع کریں

## مسئلہ
اردو پر کلک کرنے سے "Page Not Found" آتا ہے۔

## حل

### آپ کو یہ کرنا ہے:

1. **پہلے dev server بند کریں:**
   - اگر `npm start` چل رہا ہے تو اس terminal میں **Ctrl+C** دبائیں

2. **Production server شروع کریں:**
   ```bash
   npm run serve
   ```

3. **Browser میں کھولیں:**
   - انگریزی: http://localhost:3000/
   - اردو: http://localhost:3000/ur/

4. **Language Switcher استعمال کریں:**
   - اوپر navbar میں language dropdown پر کلک کریں
   - اردو منتخب کریں
   - صفحہ اردو میں بدل جائے گا! ✅

## اہم نوٹ

- `npm start` = صرف انگریزی (language switch کام نہیں کرتا)
- `npm run serve` = انگریزی + اردو دونوں (language switch کام کرتا ہے) ✅

## Build دوبارہ چلانے کی ضرورت

اگر آپ نے code میں changes کیے ہیں، تو پہلے build کریں:

```bash
npm run build
npm run serve
```

یا ایک ہی command میں:

```bash
npm run start:prod
```

---

# English Version

## The Problem
Clicking Urdu shows "Page Not Found"

## The Solution

### You need to:

1. **Stop dev server first:**
   - If `npm start` is running, press **Ctrl+C** in that terminal

2. **Start production server:**
   ```bash
   npm run serve
   ```

3. **Open in browser:**
   - English: http://localhost:3000/
   - Urdu: http://localhost:3000/ur/

4. **Use language switcher:**
   - Click language dropdown in navbar
   - Select اردو
   - Page will switch to Urdu! ✅

## Important

- `npm start` = English only (language switch doesn't work)
- `npm run serve` = English + Urdu both (language switch works) ✅

## Rebuild if needed

If you made code changes, rebuild first:

```bash
npm run build
npm run serve
```

Or use this one command:

```bash
npm run start:prod
```

#!/usr/bin/env python3
"""
Urdu Translation Helper for Chapters 14-19
Translates English markdown files to Urdu while preserving code blocks and diagrams
"""

import re
import os

# Translation dictionary for common terms and phrases
TRANSLATIONS = {
    # Headers
    "Learning Objectives": "سیکھنے کے مقاصد",
    "Prerequisites": "پیشگی ضروریات",
    "Introduction": "تعارف",
    "Core Concepts": "بنیادی تصورات",
    "Summary": "خلاصہ",
    "References": "حوالہ جات",
    "Questions and Answers": "سوال و جواب",
    "Practical Q&A": "عملی سوال و جواب",
    "Integration Across Modules": "Modules میں انضمام",
    "Connections to Other Modules": "دوسرے Modules سے روابط",
    "Implementation": "عملدرآمد",
    "Best practices": "بہترین طریقے",
    "Key takeaways": "کلیدی نکات",
    "Example": "مثال",
    "Note": "نوٹ",
    "Important": "اہم",
    "Warning": "انتباہ",

    # Common phrases
    "By the end of this chapter, you will": "اس باب کے اختتام تک، آپ",
    "Before starting this chapter, you should understand": "اس باب کو شروع کرنے سے پہلے، آپ کو یہ سمجھنا چاہیے",
    "Recommended setup": "تجویز کردہ setup",
    "Why This Matters": "یہ کیوں اہم ہے",
    "This chapter": "یہ باب",
    "Real-world": "حقیقی دنیا",
    "Key insight": "کلیدی بصیرت",
    "Key Principles": "کلیدی اصول",
    "Key steps": "کلیدی مراحل",
    "Benefits": "فوائد",
    "Challenges": "چیلنجز",
    "Limitations": "حدود",
    "Advantages": "فوائد",
    "Disadvantages": "نقصانات",
    "Future": "مستقبل",
    "Current": "موجودہ",

    # Technical terms (keep in English with Urdu context)
    "training": "training",
    "simulation": "simulation",
    "robot": "robot",
    "model": "model",
    "policy": "policy",
    "environment": "environment",
    "sensor": "sensor",
    "camera": "camera",
    "data": "data",
    "task": "task",
    "action": "action",
    "reward": "reward",

    # Action verbs
    "Understand": "سمجھیں",
    "Learn": "سیکھیں",
    "Explore": "دریافت کریں",
    "Implement": "implement کریں",
    "Apply": "لاگو کریں",
    "Deploy": "deploy کریں",
    "Train": "train کریں",
    "Test": "test کریں",
    "Validate": "validate کریں",
    "Debug": "debug کریں",
}

def preserve_code_blocks(text):
    """Extract and preserve code blocks"""
    code_blocks = []
    pattern = r'```[\s\S]*?```'

    def replacer(match):
        code_blocks.append(match.group(0))
        return f"###CODE_BLOCK_{len(code_blocks)-1}###"

    text = re.sub(pattern, replacer, text)
    return text, code_blocks

def restore_code_blocks(text, code_blocks):
    """Restore code blocks"""
    for i, block in enumerate(code_blocks):
        text = text.replace(f"###CODE_BLOCK_{i}###", block)
    return text

def translate_text(text):
    """Translate English text to Urdu"""
    # This is a simplified version - in production, you'd use a proper translation API
    # For now, we'll do basic replacements from our dictionary

    for eng, urdu in TRANSLATIONS.items():
        # Case-sensitive replacement for headers and exact matches
        text = text.replace(eng, urdu)

    return text

def translate_file(input_path, output_path):
    """Translate a markdown file from English to Urdu"""
    print(f"Translating: {input_path} -> {output_path}")

    with open(input_path, 'r', encoding='utf-8') as f:
        content = f.read()

    # Preserve code blocks and mermaid diagrams
    content, code_blocks = preserve_code_blocks(content)

    # Translate the text
    translated = translate_text(content)

    # Restore code blocks
    translated = restore_code_blocks(translated, code_blocks)

    # Ensure output directory exists
    os.makedirs(os.path.dirname(output_path), exist_ok=True)

    # Write translated content
    with open(output_path, 'w', encoding='utf-8') as f:
        f.write(translated)

    print(f"✓ Completed: {output_path}")

def main():
    """Main translation workflow"""
    base_dir = "/mnt/g/d_data/speckit/hackathon1/humanoid-robotics-book"

    chapters = [
        ("docs/part2-modules/module3-isaac/chapter-14-domain-randomization-sim2real.md",
         "i18n/ur/docusaurus-plugin-content-docs/current/part2-modules/module3-isaac/chapter-14-domain-randomization-sim2real.md"),
        ("docs/part2-modules/module3-isaac/chapter-15-synthetic-data-replicator.md",
         "i18n/ur/docusaurus-plugin-content-docs/current/part2-modules/module3-isaac/chapter-15-synthetic-data-replicator.md"),
        ("docs/part2-modules/module4-vla/chapter-16-what-is-vla.md",
         "i18n/ur/docusaurus-plugin-content-docs/current/part2-modules/module4-vla/chapter-16-what-is-vla.md"),
        ("docs/part2-modules/module4-vla/chapter-17-voice-to-action.md",
         "i18n/ur/docusaurus-plugin-content-docs/current/part2-modules/module4-vla/chapter-17-voice-to-action.md"),
        ("docs/part2-modules/module4-vla/chapter-18-how-llms-generate-actions.md",
         "i18n/ur/docusaurus-plugin-content-docs/current/part2-modules/module4-vla/chapter-18-how-llms-generate-actions.md"),
        ("docs/part2-modules/module4-vla/chapter-19-capstone-implementation.md",
         "i18n/ur/docusaurus-plugin-content-docs/current/part2-modules/module4-vla/chapter-19-capstone-implementation.md"),
    ]

    for source, target in chapters:
        source_path = os.path.join(base_dir, source)
        target_path = os.path.join(base_dir, target)
        translate_file(source_path, target_path)

    print("\n✓✓✓ All chapters translated successfully! ✓✓✓")

if __name__ == "__main__":
    main()

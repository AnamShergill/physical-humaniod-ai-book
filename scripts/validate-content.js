#!/usr/bin/env node

const fs = require('fs');
const path = require('path');

// Configuration
const CHAPTERS_DIR = './chapters';
const REQUIRED_FRONTMATTER = ['title', 'description', 'sidebar_label'];
const MARKDOWN_PATTERN = /\.md$/;

// Validation functions
function validateMarkdownFile(filePath) {
  const content = fs.readFileSync(filePath, 'utf8');
  const lines = content.split('\n');
  let frontMatter = {};
  let inFrontMatter = false;
  let frontMatterEnd = -1;

  // Extract front matter
  for (let i = 0; i < lines.length; i++) {
    if (lines[i].trim() === '---') {
      if (!inFrontMatter) {
        inFrontMatter = true;
      } else {
        frontMatterEnd = i;
        break;
      }
    } else if (inFrontMatter) {
      const colonIndex = lines[i].indexOf(':');
      if (colonIndex > 0) {
        const key = lines[i].substring(0, colonIndex).trim();
        const value = lines[i].substring(colonIndex + 1).trim();
        frontMatter[key] = value;
      }
    }
  }

  // Validate required front matter
  const missingFields = [];
  REQUIRED_FRONTMATTER.forEach(field => {
    if (!frontMatter[field]) {
      missingFields.push(field);
    }
  });

  // Check for content after front matter
  let hasContent = false;
  for (let i = frontMatterEnd + 1; i < lines.length; i++) {
    if (lines[i].trim() !== '') {
      hasContent = true;
      break;
    }
  }

  return {
    filePath,
    valid: missingFields.length === 0 && hasContent,
    missingFields,
    hasContent,
    frontMatter
  };
}

function validateDirectory(dirPath) {
  const items = fs.readdirSync(dirPath);
  const results = [];

  items.forEach(item => {
    const itemPath = path.join(dirPath, item);
    const stat = fs.statSync(itemPath);

    if (stat.isDirectory()) {
      results.push(...validateDirectory(itemPath));
    } else if (MARKDOWN_PATTERN.test(item)) {
      results.push(validateMarkdownFile(itemPath));
    }
  });

  return results;
}

// Main execution
function main() {
  console.log('Starting content validation...\n');

  const results = validateDirectory(CHAPTERS_DIR);
  let validCount = 0;
  let invalidCount = 0;

  results.forEach(result => {
    if (result.valid) {
      console.log(`✅ Valid: ${result.filePath}`);
      validCount++;
    } else {
      console.log(`❌ Invalid: ${result.filePath}`);
      if (result.missingFields.length > 0) {
        console.log(`   Missing front matter fields: ${result.missingFields.join(', ')}`);
      }
      if (!result.hasContent) {
        console.log('   Missing content after front matter');
      }
      invalidCount++;
    }
  });

  console.log('\n--- Validation Summary ---');
  console.log(`Total files: ${results.length}`);
  console.log(`Valid: ${validCount}`);
  console.log(`Invalid: ${invalidCount}`);
  console.log(`Success rate: ${results.length > 0 ? Math.round((validCount / results.length) * 100) : 0}%`);

  if (invalidCount > 0) {
    console.log('\nValidation failed! Please fix the invalid files.');
    process.exit(1);
  } else {
    console.log('\nAll files passed validation!');
    process.exit(0);
  }
}

// Run validation
if (require.main === module) {
  main();
}
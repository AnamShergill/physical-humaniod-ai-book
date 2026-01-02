import React from 'react';
import clsx from 'clsx';
import styles from './styles.module.css';

// Enhanced Section divider with visual richness
export default function SectionDivider({ className = '', ...props }) {
  return (
    <div className={clsx(styles.sectionDivider, className)} {...props}>
      <hr className={styles.dividerLine} />
      <div className={styles.dividerDots}>•••</div>
    </div>
  );
}
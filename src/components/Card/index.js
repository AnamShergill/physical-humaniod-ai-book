import React from 'react';
import clsx from 'clsx';
import styles from './styles.module.css';

// Enhanced Card component with visual depth and hierarchy
export default function Card({
  children,
  className = '',
  title,
  image,
  depth = 'medium', // 'shallow', 'medium', 'deep'
  bordered = true, // true, false
  ...props
}) {
  const cardClasses = clsx(
    styles.card,
    styles[`card--${depth}`],
    {
      [styles['card--bordered']]: bordered,
      [styles['card--borderless']]: !bordered,
    },
    className
  );

  return (
    <div className={cardClasses} {...props}>
      {title && (
        <div className={styles.cardHeader}>
          <h3 className={styles.cardTitle}>{title}</h3>
        </div>
      )}
      {image && <img src={image} alt={title || ''} className={styles.cardImage} />}
      <div className={styles.cardContent}>
        {children}
      </div>
      <div className={styles.cardDecoration}></div>
    </div>
  );
}
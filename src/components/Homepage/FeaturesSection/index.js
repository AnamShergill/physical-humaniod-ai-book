import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import styles from './styles.module.css';

const FeatureList = [
  {
    title: 'Structured Learning Path',
    description: (
      <>
        6 comprehensive chapters with 3 lessons each, following a logical progression from foundational concepts to advanced implementations.
      </>
    ),
  },
  {
    title: 'AI-Powered Assistance',
    description: (
      <>
        Integrated chatbot that answers questions based only on textbook content, providing contextual help as you learn.
      </>
    ),
  },
  {
    title: 'Multilingual Support',
    description: (
      <>
        Content available in both English and Urdu, making robotics education accessible to a broader audience.
      </>
    ),
  },
];

function Feature({Svg, title, description}) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center padding-horiz--md">
        <h3>{title}</h3>
        <p>{description}</p>
      </div>
    </div>
  );
}

export default function FeaturesSection() {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}